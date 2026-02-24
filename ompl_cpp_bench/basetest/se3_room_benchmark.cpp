#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>

#include <ompl/tools/benchmark/Benchmark.h>

#include <random>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

struct AABB {
    double xmin, xmax, ymin, ymax, zmin, zmax;
};

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

static std::vector<AABB> makeRoomWithPillars(
    double Lx, double Ly, double Lz,
    double wallThickness,
    int nPillars,
    double pillarRadius,
    double pillarHeight,
    unsigned seed
) {
    std::mt19937 rng(seed);
    const double hx = Lx/2.0, hy = Ly/2.0, hz = Lz/2.0;
    std::vector<AABB> obs;
    const double t = wallThickness;

    // 4 walls
    obs.push_back({hx - t, hx, -hy, hy, -hz, hz});        // +X
    obs.push_back({-hx, -hx + t, -hy, hy, -hz, hz});      // -X
    obs.push_back({-hx, hx, hy - t, hy, -hz, hz});        // +Y
    obs.push_back({-hx, hx, -hy, -hy + t, -hz, hz});      // -Y

    // pillars: keep margin from walls
    const double margin = t + pillarRadius + 0.15;
    std::uniform_real_distribution<double> ux(-hx + margin, hx - margin);
    std::uniform_real_distribution<double> uy(-hy + margin, hy - margin);

    const double z0 = -hz;
    const double z1 = -hz + pillarHeight;

    for (int i = 0; i < nPillars; ++i) {
        const double cx = ux(rng);
        const double cy = uy(rng);
        obs.push_back({cx - pillarRadius, cx + pillarRadius,
                       cy - pillarRadius, cy + pillarRadius,
                       z0, z1});
    }
    return obs;
}

static ob::OptimizationObjectivePtr makePathLengthObj(const ob::SpaceInformationPtr& si) {
    return std::make_shared<ob::PathLengthOptimizationObjective>(si);
}

int main(int argc, char** argv) {
    // ------------ Config ------------
    const double Lx = 6.0, Ly = 6.0, Lz = 2.5;
    const double droneRadius = 0.20;

    // Environments (match your Python idea)
    struct EnvCfg { std::string name; int nPillars; double pillarRadius; unsigned seed; };
    std::vector<EnvCfg> envs = {
        {"pillars_easy", 4, 0.20, 0},
        {"pillars_hard", 8, 0.28, 1},
        {"clutter",     12, 0.22, 2},
    };

    // Benchmark settings
    const double timeLimit = 1.0;   // seconds per run (change & rerun for 0.5/2.0, etc.)
    const int runCount = 30;        // runs per planner
    const int maxMemMB = 4096;      // safety cap

    // ------------ Create SE(3) space ------------
    auto space = std::make_shared<ob::SE3StateSpace>();
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, -Lx/2.0); bounds.setHigh(0,  Lx/2.0);
    bounds.setLow(1, -Ly/2.0); bounds.setHigh(1,  Ly/2.0);
    bounds.setLow(2, -Lz/2.0); bounds.setHigh(2,  Lz/2.0);
    space->setBounds(bounds);

    // We'll run a separate benchmark DB per environment for clarity.
    for (const auto& ec : envs) {
        const auto obstacles = makeRoomWithPillars(
            Lx, Ly, Lz,
            /*wallThickness=*/0.10,
            ec.nPillars,
            ec.pillarRadius,
            /*pillarHeight=*/2.2,
            ec.seed
        );

        og::SimpleSetup ss(space);

        // Validity checker: sphere robot vs AABB obstacles
        ss.setStateValidityChecker([&](const ob::State* st) {
            const auto* s = st->as<ob::SE3StateSpace::StateType>();
            const double x = s->getX(), y = s->getY(), z = s->getZ();
            for (const auto& b : obstacles) {
                if (sphereCollidesAABB(x, y, z, droneRadius, b)) return false;
            }
            return true;
        });

        ss.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

        // Start/goal (fixed per environment so planners are comparable)
        ob::ScopedState<ob::SE3StateSpace> start(space), goal(space);
        start->setX(-Lx/2.0 + 0.6); start->setY(-Ly/2.0 + 0.6); start->setZ(-Lz/2.0 + 0.6);
        start->rotation().setIdentity();

        goal->setX( Lx/2.0 - 0.6); goal->setY( Ly/2.0 - 0.6); goal->setZ(-Lz/2.0 + 0.6);
        goal->rotation().setIdentity();

        ss.setStartAndGoalStates(start, goal);

        // Optimize path length so "quality" is comparable for asymptotically optimal planners
        ss.getProblemDefinition()->setOptimizationObjective(makePathLengthObj(ss.getSpaceInformation()));

        // ------------ Benchmark ------------
        std::string reqName = "SE3_Room_" + ec.name + "_t" + std::to_string(timeLimit);
        ot::Benchmark b(ss, reqName);

        // Add planners
        b.addPlanner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));
        b.addPlanner(std::make_shared<og::PRMstar>(ss.getSpaceInformation()));
        b.addPlanner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
        b.addPlanner(std::make_shared<og::BITstar>(ss.getSpaceInformation()));

        ot::Benchmark::Request req;
        req.maxTime = timeLimit;
        req.maxMem = maxMemMB;          // MB
        req.runCount = runCount;
        req.displayProgress = true;
        req.saveConsoleOutput = false;

        std::cout << "\n=== Running benchmark: " << reqName << " ===\n";
        b.benchmark(req);

        const std::string dbFile = "ompl_bench_" + ec.name + "_t" + std::to_string(timeLimit) + ".log";
        b.saveResultsToFile(dbFile.c_str());
        std::cout << "Saved: " << dbFile << "\n";
    }

    return 0;
}