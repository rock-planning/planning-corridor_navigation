#include <vfh_star/VFHStar.h>
#include <iostream>
#include <vizkit3d/QtThreadedWidget.hpp>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/VFHTreeVisualization.hpp>
#include <vizkit3d/EnvironmentItemVisualizer.hpp>
#include <vizkit3d/EnvireVisualization.hpp>
#include <vizkit3d/GridVisualization.hpp>
#include <corridor_navigation/VFHServoing.hpp>


using namespace Eigen;
using namespace vfh_star;
using namespace corridor_navigation;

const int UNKNOWN = 0;
const int OBSTACLE = 1;
const int TRAVERSABLE = 2;

void markAsObstacle(envire::TraversabilityGrid::ArrayType &trData, size_t x, size_t y)
{
    trData[y][x] = OBSTACLE;
}


void addObstacle(envire::TraversabilityGrid &trGrid, const base::Pose &pose, double width, double height)
{
    envire::TraversabilityGrid::ArrayType &trData(trGrid.getGridData(envire::TraversabilityGrid::TRAVERSABILITY));
    trGrid.forEachInRectangle(pose, height, width, boost::bind(markAsObstacle, boost::ref(trData), _1, _2));
}

int main()
{

    corridor_navigation::VFHServoing servoing;
    
//     t.setSearchConf(t.getSearchConf());
    
    envire::Environment env;

    envire::TraversabilityClass unknown;
    envire::TraversabilityClass drivable(1.0);
    envire::TraversabilityClass obstacle(0.0);

    envire::TraversabilityGrid trGrid(400, 400, 0.05, 0.05, -10.0, -10.0);
    trGrid.setTraversabilityClass(UNKNOWN, unknown);
    trGrid.setTraversabilityClass(OBSTACLE, obstacle);
    trGrid.setTraversabilityClass(TRAVERSABLE, drivable);
    
    envire::FrameNode gridFrame(Eigen::Affine3d::Identity());
    env.attachItem(&trGrid);
    env.setFrameNode(&trGrid, &gridFrame);
    env.getRootNode()->addChild(&gridFrame);
    
    envire::TraversabilityGrid::ArrayType &trData(trGrid.getGridData(envire::TraversabilityGrid::TRAVERSABILITY));
    std::fill(trData.data(), trData.data() + trData.num_elements(), TRAVERSABLE);

    double robotWidth = 0.5;
    double borderWidth = 0.15;
    double obstHeight = 3.2;
    double obstWidth = 1.0;

    double offsetLeft = 0.4;
    double offsetFront = 0.4;

    base::Pose leftObstPose(Vector3d(offsetFront + obstHeight / 2.0 + borderWidth , -(robotWidth / 2.0 + borderWidth + offsetLeft + obstWidth / 2.0) ,0), Quaterniond::Identity());
    base::Pose rightObstPose(Vector3d(offsetFront + obstHeight / 2.0 + borderWidth, (robotWidth / 2.0 + borderWidth - offsetLeft + obstWidth / 2.0),0), Quaterniond::Identity());
//     size_t obstX, obstY;
//     assert(trGrid.toGrid(obstaclePos, obstX, obstY));


    addObstacle(trGrid, leftObstPose, obstWidth, obstHeight);
    addObstacle(trGrid, rightObstPose, obstWidth, obstHeight);
    
//     std::cout << "Scale " << trGrid.getScaleY() << " result " << -(obstHeight / trGrid.getScaleY() / 2.0) << std::endl;
//     
//     
//     //add obstacle in front of robot
//     for(int y = -(obstWidth / trGrid.getScaleY() / 2.0) ; y < (obstWidth / trGrid.getScaleY() / 2.0); y++)
//     {
//         for(int x = -(obstHeight / trGrid.getScaleX() / 2.0); x < (obstHeight / trGrid.getScaleX() / 2.0); x++)
//         {
//             assert((obstX + x > 0) && (obstX + x < trGrid.getCellSizeX()));
//             assert((obstY + y > 0) && (obstY + y < trGrid.getCellSizeY()));
//             trData[obstY + y][obstX + x] = OBSTACLE;
//         }
//     }
    
//     exit(0);
/*    
    for(size_t y = 0; y < trGrid.getCellSizeY(); y++)
    {
        for(size_t x = 0; x < trGrid.getCellSizeX(); x++)
        {
            int foo = trData[y][x];
            std::cout << foo << " ";
        }
        std::cout << std::endl;
    }
    */
    servoing.setNewTraversabilityGrid(&trGrid);
    
    vfh_star::TreeSearchConf conf = servoing.getSearchConf();
    conf.maxTreeSize = 100000;
    conf.stepDistance = 0.3;
    
    AngleSampleConf global;
    //5° min steps
    global.angularSamplingMin = 5 * M_PI / 360.0;
    //10° max between steps
    global.angularSamplingMax = 10 * M_PI / 180.0;
    global.angularSamplingNominalCount = 5;
    global.intervalStart = 0;
    global.intervalWidth = 2* M_PI;
    
    conf.sampleAreas.clear();
    conf.sampleAreas.push_back(global);
    
    conf.identityPositionThreshold = 0.06;
    conf.identityYawThreshold = 3*M_PI/180.0;
    servoing.setSearchConf(conf);
    
    VFHServoingConf servoingConf;
    servoingConf.vfhConfig.robotWidth = 0.5;
    servoingConf.vfhConfig.obstacleSenseRadius = 0.9;
    servoingConf.vfhConfig.histogramSize = 180;
    servoingConf.vfhConfig.narrowThreshold = 10;
    servoingConf.vfhConfig.lowThreshold = 10.0;
    
    servoingConf.baseSpeed = 0.3;
    servoingConf.speedReductionForTurning = 0.0537;

    servoingConf.pointTurnThreshold= 0.7;
    servoingConf.pointTurnSpeed= 0.7854;
    servoingConf.speedAfterPointTurn= 0.2;
    servoingConf.robotWidth= robotWidth;
    servoingConf.robotLength = 1.0;
    servoingConf.obstacleSafetyRadius= 0.05;
    servoingConf.maxInnerSpeedPenalty= 0.25;
    servoingConf.maxOuterSpeedPenalty= 0.1;
    servoingConf.driveModeChangeCost = 0.05;
    servoingConf.minimalSpeed = 0.01;
    
    servoing.setCostConf(servoingConf);
    servoing.activateDebug();
    
    base::Pose start;
    start.orientation = Eigen::Quaterniond::Identity();
    base::Angle mainHeading = base::Angle::fromDeg(0.0);
    
    base::Time startTime = base::Time::now();
    
    std::vector<base::Trajectory> trajectory;
    VFHServoing::ServoingStatus ret = servoing.getTrajectories(trajectory, start, mainHeading, 5.0, Eigen::Affine3d::Identity());
    
    base::Time endTime = base::Time::now();

    std::cout << "Starting from " << start.position.transpose() << " with heading " << start.getYaw() << " in direction of " << mainHeading << std::endl;    
    
    std::cout << "Resulting tree is " << servoing.getTree().getSize() << " took " << endTime-startTime << std::endl;

    std::cout << "Result: " << std::endl;
//     for(std::vector<base::Waypoint>::const_iterator it = trajectory.begin(); it != trajectory.end(); it++) 
//     {
//      std::cout << "  " << it->position.transpose() << std::endl;
//     }

//     for (int i = 0; i < 2; ++i)
//     {
//         std::cerr << i << std::endl;
//         std::vector<base::Waypoint> trajectory = t.getWaypoints(start, mainHeading, 5);
//     }
    
    QtThreadedWidget<vizkit3d::Vizkit3DWidget> app;
    
    vizkit3d::GridVisualization gridViz;
    vizkit3d::VFHTreeVisualization treeViz;
    envire::EnvireVisualization envViz;

    app.start();
    app.getWidget()->addPlugin(&treeViz);
    app.getWidget()->addPlugin(&envViz);
    app.getWidget()->addPlugin(&gridViz);
    
    if(servoing.getDebugTree())
    {
        treeViz.updateData(*(servoing.getDebugTree()));
    }
    treeViz.removeLeaves(false);
    
    envViz.updateData(&env);
    
    std::cout << "Tree has " << servoing.getTree().getSize() << " Nodes " << std::endl;
    
    while(app.isRunning())
        usleep(10000);
    
    
    return 0;
}

