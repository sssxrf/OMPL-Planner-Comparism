// se3_room_tour_benchmark.cpp
//
// 4-room (2x2) SE(3) multi-stage tour benchmark with deterministic seeding.
// Key property: RNG seed is set BEFORE any OMPL objects that may touch RNG
// (especially ScopedState / SimpleSetup). This avoids:
//   "Random number generation already started. Changing seed now will not lead to deterministic sampling."
//
// Outputs:
//   four_rooms_boxes.csv
//   tour_summary.csv
//   path_<planner>_run<r>.csv

#include <ompl/base/ScopedState.h>
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
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <random>
#include <string>
#include <vector>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct AABB { double xmin,xmax,ymin,ymax,zmin,zmax; };

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

// Build a 2x2 room layout (outer walls + cross walls with doors) + per-room pillars.
static std::vector<AABB> buildFourRooms(
    double W, double H, double Z,
    double wallT,
    double doorW_x_top,  double doorW_x_bot,
    double doorW_y_left, double doorW_y_right,
    int pillarsR1, int pillarsR2, int pillarsR3, int pillarsR4,
    double pillarR, double pillarH,
    unsigned seed
) {
    std::mt19937 rng(seed);
    const double hx = W/2.0, hy = H/2.0, hz = Z/2.0;
    std::vector<AABB> obs;

    // Outer walls
    obs.push_back({ hx-wallT, hx, -hy, hy, -hz, hz});
    obs.push_back({-hx, -hx+wallT, -hy, hy, -hz, hz});
    obs.push_back({-hx, hx,  hy-wallT, hy, -hz, hz});
    obs.push_back({-hx, hx, -hy, -hy+wallT, -hz, hz});

    // Internal vertical wall at x=0 split into top half and bottom half, each with its own door.
    auto addVerticalWallSegment = [&](double y0, double y1) {
        obs.push_back({-wallT/2.0, wallT/2.0, y0, y1, -hz, hz});
    };
    const double yTopCenter =  +H/4.0;
    const double yBotCenter =  -H/4.0;

    // Top half: y in [0, hy]
    {
        double doorY0 = yTopCenter - doorW_x_top/2.0;
        double doorY1 = yTopCenter + doorW_x_top/2.0;
        addVerticalWallSegment(0.0, doorY0);
        addVerticalWallSegment(doorY1, hy);
    }
    // Bottom half: y in [-hy, 0]
    {
        double doorY0 = yBotCenter - doorW_x_bot/2.0;
        double doorY1 = yBotCenter + doorW_x_bot/2.0;
        addVerticalWallSegment(-hy, doorY0);
        addVerticalWallSegment(doorY1, 0.0);
    }

    // Internal horizontal wall at y=0 split into left half and right half, each with its own door.
    auto addHorizontalWallSegment = [&](double x0, double x1) {
        obs.push_back({x0, x1, -wallT/2.0, wallT/2.0, -hz, hz});
    };
    const double xLeftCenter  = -W/4.0;
    const double xRightCenter = +W/4.0;

    // Left half: x in [-hx, 0]
    {
        double doorX0 = xLeftCenter - doorW_y_left/2.0;
        double doorX1 = xLeftCenter + doorW_y_left/2.0;
        addHorizontalWallSegment(-hx, doorX0);
        addHorizontalWallSegment(doorX1, 0.0);
    }
    // Right half: x in [0, hx]
    {
        double doorX0 = xRightCenter - doorW_y_right/2.0;
        double doorX1 = xRightCenter + doorW_y_right/2.0;
        addHorizontalWallSegment(0.0, doorX0);
        addHorizontalWallSegment(doorX1, hx);
    }

    // Per-room pillars
    auto addPillarsInRoom = [&](double xmin, double xmax, double ymin, double ymax, int nP) {
        const double margin = wallT + pillarR + 0.15;
        std::uniform_real_distribution<double> ux(xmin + margin, xmax - margin);
        std::uniform_real_distribution<double> uy(ymin + margin, ymax - margin);
        const double z0 = -hz;
        const double z1 = -hz + pillarH;

        for (int i = 0; i < nP; ++i) {
            const double cx = ux(rng);
            const double cy = uy(rng);
            obs.push_back({cx - pillarR, cx + pillarR,
                           cy - pillarR, cy + pillarR,
                           z0, z1});
        }
    };

    // Quadrants: R1 TL, R2 TR, R3 BL, R4 BR
    addPillarsInRoom(-hx, 0.0,  0.0,  hy, pillarsR1);
    addPillarsInRoom( 0.0, hx,  0.0,  hy, pillarsR2);
    addPillarsInRoom(-hx, 0.0, -hy,  0.0, pillarsR3);
    addPillarsInRoom( 0.0, hx, -hy,  0.0, pillarsR4);

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

// Plan one leg
static bool planLeg(
    og::SimpleSetup& ss,
    const ob::ScopedState<ob::SE3StateSpace>& a,
    const ob::ScopedState<ob::SE3StateSpace>& b,
    const std::string& plannerName,
    double timeLimit,
    double& solveTimeOut,
    double& lengthOut,
    og::PathGeometric& pathOut
) {
    ss.clear();
    ss.setStartAndGoalStates(a, b);
    ss.getProblemDefinition()->setOptimizationObjective(makePathLengthObj(ss.getSpaceInformation()));
    ss.setPlanner(makePlanner(plannerName, ss.getSpaceInformation()));

    auto t0 = std::chrono::high_resolution_clock::now();
    const ob::PlannerStatus solved = ss.solve(timeLimit);
    auto t1 = std::chrono::high_resolution_clock::now();

    solveTimeOut = std::chrono::duration<double>(t1 - t0).count();

    if (!solved) {
        lengthOut = std::numeric_limits<double>::quiet_NaN();
        return false;
    }

    ss.simplifySolution();
    pathOut = ss.getSolutionPath();
    lengthOut = pathOut.length();
    return true;
}

struct XYZ { double x,y,z; };

int main() {
    // -------- Environment config --------
    const double W = 10.0, H = 10.0, Z = 3.0;
    const double wallT = 0.12;
    const double droneR = 0.20;

    // Door widths (hallway difficulty knobs)
    const double doorW_x_top   = 1.20;  // R1<->R2 (easy)
    const double doorW_x_bot   = 0.60;  // R3<->R4 (hard)
    const double doorW_y_left  = 0.90;  // R1<->R3 (medium)
    const double doorW_y_right = 0.50;  // R2<->R4 (hard)

    // Obstacles per room
    const double pillarR = 0.25, pillarH = 2.6;
    const int pR1 = 2, pR2 = 6, pR3 = 4, pR4 = 8;

    const auto obstacles = buildFourRooms(
        W, H, Z, wallT,
        doorW_x_top, doorW_x_bot, doorW_y_left, doorW_y_right,
        pR1, pR2, pR3, pR4,
        pillarR, pillarH,
        /*seed=*/42
    );

    writeBoxesCSV("four_rooms_boxes.csv", obstacles);

    // -------- SE(3) space (bounds only; safe to build once) --------
    auto space = std::make_shared<ob::SE3StateSpace>();
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, -W/2.0); bounds.setHigh(0,  W/2.0);
    bounds.setLow(1, -H/2.0); bounds.setHigh(1,  H/2.0);
    bounds.setLow(2, -Z/2.0); bounds.setHigh(2,  Z/2.0);
    space->setBounds(bounds);

    // -------- Waypoint coordinates ONLY (no ScopedState yet) --------
    const double z_wp = -Z/2.0 + 1.2;
    const double hx = W/2.0, hy = H/2.0;

    const XYZ Sxyz  { -hx + 1.0,  +hy - 1.0, z_wp }; // start in R1
    const XYZ W1xyz { -W/4.0,     +H/4.0,    z_wp }; // R1
    const XYZ W2xyz { +W/4.0,     +H/4.0,    z_wp }; // R2
    const XYZ W3xyz { +W/4.0,     -H/4.0,    z_wp }; // R4
    const XYZ W4xyz { -W/4.0,     -H/4.0,    z_wp }; // R3

    // -------- Planners + experiment settings --------
    const std::vector<std::string> planners = {"RRTConnect", "PRMstar", "RRTstar", "BITstar"};
    const int runs = 20;
    const double timeLimit = 1.0;     // per leg
    const unsigned baseSeed = 12345;

    std::ofstream summary("tour_summary.csv");
    summary << "planner,run,success,total_time_s,total_length\n";

    for (const auto& planner : planners) {
        const unsigned plannerHash = static_cast<unsigned>(std::hash<std::string>{}(planner));

        for (int r = 0; r < runs; ++r) {
            // ---- CRITICAL: seed BEFORE constructing SimpleSetup / ScopedState ----
            const unsigned seed = baseSeed + 1000u * static_cast<unsigned>(r) + (plannerHash % 1000u);
            ompl::RNG::setSeed(seed);

            // Now create OMPL objects that might touch RNG
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

            // ScopedStates constructed AFTER seeding
            ob::ScopedState<ob::SE3StateSpace> S(space), W1(space), W2(space), W3(space), W4(space);
            setSE3(S,  Sxyz.x,  Sxyz.y,  Sxyz.z);
            setSE3(W1, W1xyz.x, W1xyz.y, W1xyz.z);
            setSE3(W2, W2xyz.x, W2xyz.y, W2xyz.z);
            setSE3(W3, W3xyz.x, W3xyz.y, W3xyz.z);
            setSE3(W4, W4xyz.x, W4xyz.y, W4xyz.z);

            std::vector<ob::ScopedState<ob::SE3StateSpace>> tour = {S, W1, W2, W3, W4, S};

            bool ok = true;
            double totalTime = 0.0;
            double totalLen = 0.0;

            const std::string pathFile = "path_" + planner + "_run" + std::to_string(r) + ".csv";
            { std::ofstream clear(pathFile); } // truncate

            bool wroteHeader = false;

            for (std::size_t i = 0; i + 1 < tour.size(); ++i) {
                double legTime = 0.0, legLen = 0.0;
                og::PathGeometric legPath(ss.getSpaceInformation());

                if (!planLeg(ss, tour[i], tour[i+1], planner, timeLimit, legTime, legLen, legPath)) {
                    ok = false;
                    break;
                }

                totalTime += legTime;
                totalLen  += legLen;

                appendPathCSV(pathFile, legPath, !wroteHeader);
                wroteHeader = true;
            }

            summary << planner << "," << r << "," << (ok ? 1 : 0) << ","
                    << std::fixed << std::setprecision(6) << totalTime << ","
                    << (ok ? totalLen : std::numeric_limits<double>::quiet_NaN()) << "\n";

            std::cout << planner << " run " << r
                      << " seed=" << seed
                      << " success=" << (ok ? "YES" : "NO")
                      << " totalTime=" << totalTime
                      << " totalLen=" << (ok ? totalLen : -1.0)
                      << "\n";
        }
    }

    std::cout << "Wrote four_rooms_boxes.csv, tour_summary.csv, and path_<planner>_run*.csv\n";
    return 0;
}