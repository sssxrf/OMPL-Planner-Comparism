// se3_room_tour_one_run.cpp
//
// One tour per PROCESS (deterministic seeding per process).
//
// NEW behavior requested:
// - If fixed waypoints are invalid for the sampled obstacle layout,
//   resample ONLY the obstacles (same env_seed base) until all waypoints are valid,
//   up to --max_resample attempts.
//
// Usage:
//   ./se3_room_tour_one_run --planner RRTConnect --seed 12345 --env_seed 42 --run 0 --outdir results
//
// Optional:
//   --max_resample 50   (default 50)
//   --env_resample_stride 1 (default 1)  // how obstacle RNG seed increments per attempt
//
// Outputs:
//   <outdir>/four_rooms_boxes_env<env_seed>.csv            (final obstacles used; overwritten)
//   <outdir>/four_rooms_meta_env<env_seed>.txt            (records obstacle_seed + attempts)
//   <outdir>/path_<planner>_env<env_seed>_run<run>_seed<seed>.csv
//   <outdir>/tour_summary.csv                             (append)

#include <ompl/base/ScopedState.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>

#include <ompl/util/RandomNumbers.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <random>
#include <string>
#include <vector>
#include <iostream>

// Simple filesystem-lite helpers (no <filesystem>)
#include <sys/stat.h>
#include <errno.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct AABB { double xmin,xmax,ymin,ymax,zmin,zmax; };
struct XYZ  { double x,y,z; };
struct XY   { double x,y; };

static bool fileExists(const std::string& path) {
    std::ifstream f(path);
    return f.good();
}

static void ensureDir(const std::string& dir) {
    if (dir.empty()) return;
    struct stat st;
    if (stat(dir.c_str(), &st) == 0) {
        if (S_ISDIR(st.st_mode)) return;
        throw std::runtime_error("Path exists but is not a directory: " + dir);
    }
    if (mkdir(dir.c_str(), 0755) != 0) {
        if (errno != EEXIST) throw std::runtime_error("Failed to create directory: " + dir);
    }
}

static std::string joinPath(const std::string& a, const std::string& b) {
    if (a.empty()) return b;
    if (!a.empty() && a.back() == '/') return a + b;
    return a + "/" + b;
}

static inline double clamp(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
}

static inline double sqDistPointAABB(double px, double py, double pz, const AABB& b) {
    const double cx = clamp(px, b.xmin, b.xmax);
    const double cy = clamp(py, b.ymin, b.ymax);
    const double cz = clamp(pz, b.zmin, b.zmax);
    const double dx = px - cx, dy = py - cy, dz = pz - cz;
    return dx*dx + dy*dy + dz*dz;
}

static inline bool sphereCollidesAABB(double px, double py, double pz, double r, const AABB& b) {
    return sqDistPointAABB(px, py, pz, b) <= r*r;
}

// Waypoint keepout test against an AABB in XY only (z ignored).
static inline bool aabbIntersectsKeepoutXY(const AABB& box, const XY& w, double keepoutR) {
    const double cx = clamp(w.x, box.xmin, box.xmax);
    const double cy = clamp(w.y, box.ymin, box.ymax);
    const double dx = w.x - cx;
    const double dy = w.y - cy;
    return (dx*dx + dy*dy) <= keepoutR * keepoutR;
}

static void writeBoxesCSV(const std::string& filename, const std::vector<AABB>& obs) {
    std::ofstream out(filename);
    out << "xmin,xmax,ymin,ymax,zmin,zmax\n";
    for (const auto& b : obs) {
        out << b.xmin << "," << b.xmax << ","
            << b.ymin << "," << b.ymax << ","
            << b.zmin << "," << b.zmax << "\n";
    }
}

static void appendPathCSV(const std::string& filename, const og::PathGeometric& path, bool writeHeaderIfNew) {
    std::ofstream out(filename, std::ios::app);
    if (writeHeaderIfNew) out << "x,y,z\n";
    for (std::size_t i = 0; i < path.getStateCount(); ++i) {
        const auto* s = path.getState(i)->as<ob::SE3StateSpace::StateType>();
        out << s->getX() << "," << s->getY() << "," << s->getZ() << "\n";
    }
}

static ob::OptimizationObjectivePtr makePathLengthObj(const ob::SpaceInformationPtr& si) {
    return std::make_shared<ob::PathLengthOptimizationObjective>(si);
}

static double pathLengthXYZ(const og::PathGeometric& path) {
    if (path.getStateCount() < 2) return 0.0;
    double sum = 0.0;
    for (std::size_t i = 1; i < path.getStateCount(); ++i) {
        const auto* a = path.getState(i - 1)->as<ob::SE3StateSpace::StateType>();
        const auto* b = path.getState(i)->as<ob::SE3StateSpace::StateType>();
        const double dx = b->getX() - a->getX();
        const double dy = b->getY() - a->getY();
        const double dz = b->getZ() - a->getZ();
        sum += std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    return sum;
}

static std::vector<AABB> buildFourRooms(
    double W, double H, double Z,
    double wallT,
    double doorW_x_top,  double doorW_x_bot,
    double doorW_y_left, double doorW_y_right,
    int pillarsR1, int pillarsR2, int pillarsR3, int pillarsR4,
    double pillarR, double pillarH,
    unsigned seed,
    const std::vector<XY>& waypointXY,
    double keepoutR
){
    std::mt19937 rng(seed);
    const double hx = W/2.0, hy = H/2.0, hz = Z/2.0;
    std::vector<AABB> obs;

    // Outer walls
    obs.push_back({ hx-wallT, hx, -hy, hy, -hz, hz});
    obs.push_back({-hx, -hx+wallT, -hy, hy, -hz, hz});
    obs.push_back({-hx, hx,  hy-wallT, hy, -hz, hz});
    obs.push_back({-hx, hx, -hy, -hy+wallT, -hz, hz});

    // Vertical wall at x=0 with two doors (top/bottom halves)
    auto addVerticalWallSegment = [&](double y0, double y1) {
        obs.push_back({-wallT/2.0, wallT/2.0, y0, y1, -hz, hz});
    };
    const double yTopCenter = +H/4.0;
    const double yBotCenter = -H/4.0;

    // Top half [0, hy]
    {
        const double doorY0 = yTopCenter - doorW_x_top/2.0;
        const double doorY1 = yTopCenter + doorW_x_top/2.0;
        addVerticalWallSegment(0.0, doorY0);
        addVerticalWallSegment(doorY1, hy);
    }
    // Bottom half [-hy, 0]
    {
        const double doorY0 = yBotCenter - doorW_x_bot/2.0;
        const double doorY1 = yBotCenter + doorW_x_bot/2.0;
        addVerticalWallSegment(-hy, doorY0);
        addVerticalWallSegment(doorY1, 0.0);
    }

    // Horizontal wall at y=0 with two doors (left/right halves)
    auto addHorizontalWallSegment = [&](double x0, double x1) {
        obs.push_back({x0, x1, -wallT/2.0, wallT/2.0, -hz, hz});
    };
    const double xLeftCenter  = -W/4.0;
    const double xRightCenter = +W/4.0;

    // Left half [-hx, 0]
    {
        const double doorX0 = xLeftCenter - doorW_y_left/2.0;
        const double doorX1 = xLeftCenter + doorW_y_left/2.0;
        addHorizontalWallSegment(-hx, doorX0);
        addHorizontalWallSegment(doorX1, 0.0);
    }
    // Right half [0, hx]
    {
        const double doorX0 = xRightCenter - doorW_y_right/2.0;
        const double doorX1 = xRightCenter + doorW_y_right/2.0;
        addHorizontalWallSegment(0.0, doorX0);
        addHorizontalWallSegment(doorX1, hx);
    }

    // Pillars in each quadrant, rejecting pillars that violate keepout around waypoints.
    auto addPillarsInRoom = [&](double xmin, double xmax, double ymin, double ymax, int nP) {
        const double margin = wallT + pillarR + 0.15;
        std::uniform_real_distribution<double> ux(xmin + margin, xmax - margin);
        std::uniform_real_distribution<double> uy(ymin + margin, ymax - margin);
        const double z0 = -hz;
        const double z1 = -hz + pillarH;

        int placed = 0;
        int tries = 0;
        const int maxTries = 20000;

        while (placed < nP && tries < maxTries) {
            ++tries;
            const double cx = ux(rng);
            const double cy = uy(rng);

            AABB pillarBox{
                cx - pillarR, cx + pillarR,
                cy - pillarR, cy + pillarR,
                z0, z1
            };

            bool ok = true;
            for (const auto& w : waypointXY) {
                if (aabbIntersectsKeepoutXY(pillarBox, w, keepoutR)) {
                    ok = false;
                    break;
                }
            }
            if (!ok) continue;

            obs.push_back(pillarBox);
            ++placed;
        }

        if (placed < nP) {
            std::cerr << "Warning: only placed " << placed << "/" << nP
                      << " pillars in room due to keepout constraints.\n";
        }
    };

    addPillarsInRoom(-hx, 0.0,  0.0,  hy, pillarsR1); // R1
    addPillarsInRoom( 0.0, hx,  0.0,  hy, pillarsR2); // R2
    addPillarsInRoom(-hx, 0.0, -hy,  0.0, pillarsR3); // R3
    addPillarsInRoom( 0.0, hx, -hy,  0.0, pillarsR4); // R4

    return obs;
}

static void setSE3(ob::ScopedState<ob::SE3StateSpace>& st, double x, double y, double z) {
    st->setX(x); st->setY(y); st->setZ(z);
    st->rotation().setIdentity();
}

static ob::PlannerPtr makePlanner(const std::string& name, const ob::SpaceInformationPtr& si) {
    if (name == "RRTConnect") return std::make_shared<og::RRTConnect>(si);
    if (name == "PRMstar")    return std::make_shared<og::PRMstar>(si);
    if (name == "RRTstar")    return std::make_shared<og::RRTstar>(si);
    if (name == "BITstar")    return std::make_shared<og::BITstar>(si);
    throw std::runtime_error("Unknown planner: " + name);
}

enum class TerminationMode {
    Budget,
    FirstSolution
};

static TerminationMode parseTerminationMode(const std::string& s) {
    if (s == "budget") return TerminationMode::Budget;
    if (s == "first_solution") return TerminationMode::FirstSolution;
    throw std::runtime_error("Unknown --termination_mode value: " + s + " (expected: budget | first_solution)");
}

static bool planLeg(
    og::SimpleSetup& ss,
    const ob::ScopedState<ob::SE3StateSpace>& a,
    const ob::ScopedState<ob::SE3StateSpace>& b,
    const std::string& plannerName,
    TerminationMode termMode,
    double timeLimit,
    double& solveTimeOut,
    double& simplifyTimeOut,
    double& lengthSE3Out,
    double& lengthXYZOut,
    og::PathGeometric& pathOut
) {
    ss.clear();
    ss.setStartAndGoalStates(a, b);
    ss.getProblemDefinition()->setOptimizationObjective(makePathLengthObj(ss.getSpaceInformation()));
    ss.setPlanner(makePlanner(plannerName, ss.getSpaceInformation()));

    auto t0 = std::chrono::high_resolution_clock::now();
    ob::PlannerStatus solved = ob::PlannerStatus::UNKNOWN;
    if (termMode == TerminationMode::Budget) {
        solved = ss.solve(timeLimit);
    } else {
        const auto timed = ob::timedPlannerTerminationCondition(timeLimit);
        const auto exact = ob::exactSolnPlannerTerminationCondition(ss.getProblemDefinition());
        const auto ptc = ob::plannerOrTerminationCondition(timed, exact);
        solved = ss.solve(ptc);
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    solveTimeOut = std::chrono::duration<double>(t1 - t0).count();

    if (!solved) {
        simplifyTimeOut = 0.0;
        lengthSE3Out = std::numeric_limits<double>::quiet_NaN();
        lengthXYZOut = std::numeric_limits<double>::quiet_NaN();
        return false;
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    ss.simplifySolution();
    auto t3 = std::chrono::high_resolution_clock::now();
    simplifyTimeOut = std::chrono::duration<double>(t3 - t2).count();

    pathOut = ss.getSolutionPath();
    lengthSE3Out = pathOut.length();
    lengthXYZOut = pathLengthXYZ(pathOut);
    return true;
}

// Validate a waypoint against bounds and obstacles (sphere vs AABB)
static bool waypointValid(
    const XYZ& p,
    double W, double H, double Z,
    const std::vector<AABB>& obstacles,
    double droneR
) {
    const double hx = W/2.0, hy = H/2.0, hz = Z/2.0;

    // Bounds check (include drone radius margin)
    if (p.x < -hx + droneR || p.x > hx - droneR) return false;
    if (p.y < -hy + droneR || p.y > hy - droneR) return false;
    if (p.z < -hz + droneR || p.z > hz - droneR) return false;

    for (const auto& b : obstacles) {
        if (sphereCollidesAABB(p.x, p.y, p.z, droneR, b)) return false;
    }
    return true;
}

// Tiny arg parser
static std::string getArg(int argc, char** argv, const std::string& key, const std::string& def="") {
    for (int i = 1; i + 1 < argc; ++i)
        if (argv[i] == key) return argv[i+1];
    return def;
}

static bool hasArg(int argc, char** argv, const std::string& key) {
    for (int i = 1; i < argc; ++i)
        if (argv[i] == key) return true;
    return false;
}

int main(int argc, char** argv) {
    if (hasArg(argc, argv, "--help")) {
        std::cout << "Usage: ./se3_room_tour_one_run --planner NAME --seed UINT --env_seed UINT --run INT --outdir DIR "
                     "[--max_resample INT] [--env_resample_stride UINT] "
                     "[--termination_mode budget|first_solution] [--time_limit_per_leg SEC]\n";
        return 0;
    }

    const std::string planner = getArg(argc, argv, "--planner", "RRTConnect");
    const unsigned seed = static_cast<unsigned>(std::stoul(getArg(argc, argv, "--seed", "12345")));
    const unsigned envSeed = static_cast<unsigned>(std::stoul(getArg(argc, argv, "--env_seed", "42")));
    const int runId = std::stoi(getArg(argc, argv, "--run", "0"));
    const std::string outdir = getArg(argc, argv, "--outdir", "results");

    const int maxResample = std::stoi(getArg(argc, argv, "--max_resample", "50"));
    const unsigned envResampleStride = static_cast<unsigned>(std::stoul(getArg(argc, argv, "--env_resample_stride", "1")));
    const TerminationMode termMode = parseTerminationMode(getArg(argc, argv, "--termination_mode", "budget"));
    const double timeLimit = std::stod(getArg(argc, argv, "--time_limit_per_leg", "1.0"));

    ensureDir(outdir);

    // IMPORTANT: seed once per process (planner randomness)
    ompl::RNG::setSeed(seed);

    // -------- Environment config --------
    const double W = 10.0, H = 10.0, Z = 3.0;
    const double wallT = 0.12;
    const double droneR = 0.20;

    const double doorW_x_top   = 1.20;
    const double doorW_x_bot   = 0.60;
    const double doorW_y_left  = 0.90;
    const double doorW_y_right = 0.50;

    const double pillarR = 0.25, pillarH = 2.6;
    const int pR1 = 2, pR2 = 6, pR3 = 4, pR4 = 8;

    const double hx = W/2.0, hy = H/2.0;

    // Fixed waypoint XY (keepout will avoid placing pillars near these)
    std::vector<XY> wpsXY = {
        { -hx + 1.0, +hy - 1.0 }, // S
        { -W/4.0,    +H/4.0    }, // W1
        { +W/4.0,    +H/4.0    }, // W2
        { +W/4.0,    -H/4.0    }, // W3
        { -W/4.0,    -H/4.0    }, // W4
    };

    // Keepout around waypoints for pillar placement
    const double keepoutR = pillarR + droneR + 0.10;

    // Flight altitude
    const double z_wp = -Z/2.0 + 1.2;

    // Fixed 3D waypoints
    const XYZ Sxyz  { -hx + 1.0,  +hy - 1.0, z_wp };
    const XYZ W1xyz { -W/4.0,     +H/4.0,    z_wp };
    const XYZ W2xyz { +W/4.0,     +H/4.0,    z_wp };
    const XYZ W3xyz { +W/4.0,     -H/4.0,    z_wp };
    const XYZ W4xyz { -W/4.0,     -H/4.0,    z_wp };

    struct NamedWP { const char* name; XYZ p; };
    const std::vector<NamedWP> wps3d = {
        {"S",  Sxyz}, {"W1", W1xyz}, {"W2", W2xyz}, {"W3", W3xyz}, {"W4", W4xyz}
    };

    // -------- Resample obstacles until waypoints are valid --------
    std::vector<AABB> obstacles;
    unsigned obstacleSeedUsed = envSeed;
    int attemptUsed = -1;

    for (int attempt = 0; attempt < maxResample; ++attempt) {
        // Deterministic sequence of obstacle seeds derived from envSeed
        // (envSeed fixed conceptually; attempt changes the obstacle RNG stream)
        const std::uint64_t obsSeed64 =
            static_cast<std::uint64_t>(envSeed) * 1000000ULL +
            static_cast<std::uint64_t>(attempt) * static_cast<std::uint64_t>(envResampleStride);
        const unsigned obsSeed = static_cast<unsigned>(obsSeed64 & 0xFFFFFFFFu);

        auto cand = buildFourRooms(
            W, H, Z, wallT,
            doorW_x_top, doorW_x_bot, doorW_y_left, doorW_y_right,
            pR1, pR2, pR3, pR4,
            pillarR, pillarH,
            obsSeed,
            wpsXY,
            keepoutR
        );

        bool allValid = true;
        for (const auto& w : wps3d) {
            if (!waypointValid(w.p, W, H, Z, cand, droneR)) {
                allValid = false;
                break;
            }
        }

        if (allValid) {
            obstacles = std::move(cand);
            obstacleSeedUsed = obsSeed;
            attemptUsed = attempt;
            break;
        }
    }

    if (attemptUsed < 0) {
        std::cerr << "Failed: could not generate a valid environment after "
                  << maxResample << " obstacle resamples for env_seed=" << envSeed << "\n";
        std::cerr << "Try: increase --max_resample, increase keepoutR (in code), move waypoints, or raise z_wp.\n";
        return 2;
    }

    // Write boxes for this environment (final obstacles used; overwrite to match actual map)
    const std::string boxesFile = joinPath(outdir, "four_rooms_boxes_env" + std::to_string(envSeed) + ".csv");
    writeBoxesCSV(boxesFile, obstacles);

    // Write meta info
    {
        const std::string metaFile = joinPath(outdir, "four_rooms_meta_env" + std::to_string(envSeed) + ".txt");
        std::ofstream m(metaFile);
        m << "env_seed=" << envSeed << "\n";
        m << "obstacle_seed_used=" << obstacleSeedUsed << "\n";
        m << "resample_attempt_used=" << attemptUsed << "\n";
        m << "keepoutR=" << keepoutR << "\n";
        m << "z_wp=" << z_wp << "\n";
    }

    // -------- SE(3) space --------
    auto space = std::make_shared<ob::SE3StateSpace>();
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, -W/2.0); bounds.setHigh(0,  W/2.0);
    bounds.setLow(1, -H/2.0); bounds.setHigh(1,  H/2.0);
    bounds.setLow(2, -Z/2.0); bounds.setHigh(2,  Z/2.0);
    space->setBounds(bounds);

    // -------- One tour solve --------
    og::SimpleSetup ss(space);

    ss.setStateValidityChecker([&](const ob::State* st) {
        const auto* s = st->as<ob::SE3StateSpace::StateType>();
        const double x = s->getX(), y = s->getY(), z = s->getZ();
        for (const auto& b : obstacles) {
            if (sphereCollidesAABB(x, y, z, droneR, b)) return false;
        }
        return true;
    });
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    ob::ScopedState<ob::SE3StateSpace> S(space), W1(space), W2(space), W3(space), W4(space);
    setSE3(S,  Sxyz.x,  Sxyz.y,  Sxyz.z);
    setSE3(W1, W1xyz.x, W1xyz.y, W1xyz.z);
    setSE3(W2, W2xyz.x, W2xyz.y, W2xyz.z);
    setSE3(W3, W3xyz.x, W3xyz.y, W3xyz.z);
    setSE3(W4, W4xyz.x, W4xyz.y, W4xyz.z);

    std::vector<ob::ScopedState<ob::SE3StateSpace>> tour = {S, W1, W2, W3, W4, S};

    bool ok = true;
    double totalSolveTime = 0.0;
    double totalSimplifyTime = 0.0;
    double totalTime = 0.0;
    double totalLenSE3  = 0.0;
    double totalLenXYZ  = 0.0;

    const std::string pathFile = joinPath(
        outdir,
        "path_" + planner + "_env" + std::to_string(envSeed) +
        "_run" + std::to_string(runId) +
        "_seed" + std::to_string(seed) + ".csv"
    );
    { std::ofstream clear(pathFile); } // truncate

    bool wroteHeader = false;

    for (std::size_t i = 0; i + 1 < tour.size(); ++i) {
        double legSolveTime = 0.0, legSimplifyTime = 0.0;
        double legLenSE3 = 0.0, legLenXYZ = 0.0;
        og::PathGeometric legPath(ss.getSpaceInformation());

        if (!planLeg(ss, tour[i], tour[i+1], planner, termMode, timeLimit,
                     legSolveTime, legSimplifyTime, legLenSE3, legLenXYZ, legPath)) {
            totalSolveTime += legSolveTime;
            totalSimplifyTime += legSimplifyTime;
            totalTime = totalSolveTime + totalSimplifyTime;
            ok = false;
            break;
        }

        totalSolveTime += legSolveTime;
        totalSimplifyTime += legSimplifyTime;
        totalTime = totalSolveTime + totalSimplifyTime;
        totalLenSE3 += legLenSE3;
        totalLenXYZ += legLenXYZ;

        appendPathCSV(pathFile, legPath, !wroteHeader);
        wroteHeader = true;
    }

    // Append summary row
    const std::string summaryFile = joinPath(outdir, "tour_summary.csv");
    const bool summaryExists = fileExists(summaryFile);
    {
        std::ofstream out(summaryFile, std::ios::app);
        if (!summaryExists) {
            out << "planner,env_seed,run,seed,obstacle_seed,resample_attempt,termination_mode,time_limit_per_leg,"
                   "success,solve_time_s,simplify_time_s,total_time_s,total_length_se3,total_length_xyz\n";
        }
        const char* termModeStr = (termMode == TerminationMode::Budget ? "budget" : "first_solution");
        out << planner << "," << envSeed << "," << runId << "," << seed << ","
            << obstacleSeedUsed << "," << attemptUsed << ","
            << termModeStr << "," << std::fixed << std::setprecision(3) << timeLimit << ","
            << (ok ? 1 : 0) << ","
            << std::fixed << std::setprecision(6) << totalSolveTime << ","
            << std::fixed << std::setprecision(6) << totalSimplifyTime << ","
            << std::fixed << std::setprecision(6) << totalTime << ","
            << (ok ? totalLenSE3 : std::numeric_limits<double>::quiet_NaN()) << ","
            << (ok ? totalLenXYZ : std::numeric_limits<double>::quiet_NaN()) << "\n";
    }

    std::cout << planner
              << " env=" << envSeed
              << " run=" << runId
              << " seed=" << seed
              << " obstacle_seed=" << obstacleSeedUsed
              << " attempt=" << attemptUsed
              << " term_mode=" << (termMode == TerminationMode::Budget ? "budget" : "first_solution")
              << " success=" << (ok ? "YES" : "NO")
              << " solveTime=" << totalSolveTime
              << " simplifyTime=" << totalSimplifyTime
              << " totalTime=" << totalTime
              << " totalLenSE3=" << (ok ? totalLenSE3 : -1.0)
              << " totalLenXYZ=" << (ok ? totalLenXYZ : -1.0) << "\n";

    return 0;
}
