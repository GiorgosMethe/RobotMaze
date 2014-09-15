#include "HCUBE_Defines.h"

#include "Experiments/HCUBE_RobotMaze.h"

using namespace NEAT;

namespace HCUBE
{
RobotMaze::RobotMaze(string _experimentName,int _threadID)
    :
      Experiment(_experimentName,_threadID)
{}

NEAT::GeneticPopulation* RobotMaze::createInitialPopulation(int populationSize)
{
    GeneticPopulation *population = new GeneticPopulation();
    vector<GeneticNodeGene> genes;

    genes.push_back(GeneticNodeGene("Bias","NetworkSensor",0,false));
    genes.push_back(GeneticNodeGene("rangeFinderN","NetworkSensor",0,false));
    genes.push_back(GeneticNodeGene("rangeFinderNE","NetworkSensor",0,false));
    genes.push_back(GeneticNodeGene("rangeFinderNW","NetworkSensor",0,false));
    genes.push_back(GeneticNodeGene("rangeFinderE","NetworkSensor",0,false));
    genes.push_back(GeneticNodeGene("rangeFinderW","NetworkSensor",0,false));
    genes.push_back(GeneticNodeGene("rangeFinderS","NetworkSensor",0,false));

    genes.push_back(GeneticNodeGene("radarN","NetworkSensor",0,false));
    genes.push_back(GeneticNodeGene("radarS","NetworkSensor",0,false));
    genes.push_back(GeneticNodeGene("radarE","NetworkSensor",0,false));
    genes.push_back(GeneticNodeGene("radarW","NetworkSensor",0,false));

    genes.push_back(GeneticNodeGene("OutputLR","NetworkOutputNode",1,false,ACTIVATION_FUNCTION_SIGMOID));
    genes.push_back(GeneticNodeGene("OutputFB","NetworkOutputNode",1,false,ACTIVATION_FUNCTION_SIGMOID));

    for (int a=0;a<populationSize;a++)
    {
        shared_ptr<GeneticIndividual> individual(new GeneticIndividual(genes,true,1.0));

        for (int b=0;b<0;b++)
        {
            individual->testMutate();
        }

        population->addIndividual(individual);
    }

    cout << "Finished creating population\n";
    return population;
}

void RobotMaze::processGroup(shared_ptr<NEAT::GeneticGeneration> generation)
{
    int genNum = generation->getGenerationNumber() + 1;
    char buffer2 [50];
    sprintf(buffer2, "%04i", genNum);

    for(int z=0;z< group.size();z++)
    {
        shared_ptr<NEAT::GeneticIndividual> individual = group[z];
        char buffer1 [50]; sprintf(buffer1, "%04i", genNum);
        std::ostringstream tmp;
        tmp << "Softbot" << "--Gen_" << buffer1 << "--Ind_" << individual;
        string individualID = tmp.str();
        cout << individualID << endl;
        processEvaluation(individual, individualID, genNum);
    }
}

void RobotMaze::processEvaluation(shared_ptr<NEAT::GeneticIndividual> individual, string individualID, int genNum)
{
    NEAT::FastNetwork<double> network = individual->spawnFastPhenotypeStack<double>();

    robot _robot;
    point goal;
    vector<lineSegment> walls;
    readMaze(_robot, goal, walls);
    boost::timer timer;
    for (int var = 0; var < 1000; ++var) {
        _robot.updateSensors(goal, walls);

        network.reinitialize();
        network.setValue("rangeFinderN",(double)_robot.rangeFinderSensor[rangeFinderN]);
        network.setValue("rangeFinderNE",(double)_robot.rangeFinderSensor[rangeFinderNE]);
        network.setValue("rangeFinderNW",(double)_robot.rangeFinderSensor[rangeFinderNW]);
        network.setValue("rangeFinderE",(double)_robot.rangeFinderSensor[rangeFinderE]);
        network.setValue("rangeFinderW",(double)_robot.rangeFinderSensor[rangeFinderW]);
        network.setValue("rangeFinderS",(double)_robot.rangeFinderSensor[rangeFinderS]);

        network.setValue("radarN", (double)_robot.radarSensor[radarN]);
        network.setValue("radarS", (double)_robot.radarSensor[radarS]);
        network.setValue("radarW", (double)_robot.radarSensor[radarW]);
        network.setValue("radarE", (double)_robot.radarSensor[radarE]);

        network.update();

        double turn = network.getValue("OutputLR");
        double forwBack = network.getValue("OutputFB");

        drive(_robot, walls, 0.1 * turn, 0.01 * forwBack);
//        drawMaze(_robot, goal, walls);
    }
    //    drawMaze(_robot, goal, walls);
    std::cout << "Simulation took: " << timer.elapsed() << " Seconds to complete.\n";
    double fitness = 1 / (_robot.position.distance(goal) + 10e-5);
    cout << "Fitness: " << fitness << endl;
    individual->setFitness(fitness);
    individual->behavior = cv::Point2d(_robot.position.x, _robot.position.y);
    return;
}


void RobotMaze::drive(robot &_robot, vector<lineSegment> walls, double turn, double d)
{
    _robot.position.theta -= turn;
    double distance = d;

    double minDist = 100.0;
    double angle = (distance > 0) ? 0 : M_PI;
    for (int i = 0; i < walls.size(); ++i) {
        point a;
        if (walls.at(i).intersection(_robot.position, remainder(_robot.position.theta + angle, 2*M_PI), a))
        {
            double tmpDis = _robot.position.distance(a);
            if(tmpDis < minDist)
            {
                minDist = tmpDis;
            }
        }
    }

    distance = min(distance, (minDist - _robot.radius));

    distance = min(distance, (minDist - _robot.radius));

    point newPosition = _robot.position.pointToDis(distance);
    bool isValid = true;
    for (int i = 0; i < walls.size(); ++i) {
        if ((walls.at(i).pointDist(newPosition, true)) < _robot.radius) isValid = false;
    }
    if(isValid)
    {
        _robot.position.x = newPosition.x;
        _robot.position.y = newPosition.y;
    }
    else
    {
        double dx = newPosition.x - _robot.position.x;
        double dy = newPosition.y - _robot.position.y;
        point newPosition; newPosition.x = _robot.position.x + dx; newPosition.y = _robot.position.y;
        bool isValid = true;
        for (int i = 0; i < walls.size(); ++i) {
            if ((walls.at(i).pointDist(newPosition, true)) < _robot.radius) isValid = false;
        }
        if(isValid)
        {
            _robot.position.x = newPosition.x;
            _robot.position.y = newPosition.y;
        }
        else
        {
            isValid = true;
            point newPosition; newPosition.y = _robot.position.y + dy; newPosition.x = _robot.position.x;
            bool isValid = true;
            for (int i = 0; i < walls.size(); ++i) {
                if ((walls.at(i).pointDist(newPosition, true)) < _robot.radius) isValid = false;
            }
            if(isValid)
            {
                _robot.position.x = newPosition.x;
                _robot.position.y = newPosition.y;
            }
        }
    }
    _robot.position.theta = remainder(_robot.position.theta, 2*M_PI);
}



void RobotMaze::drawMaze(robot _robot, point goal, vector<lineSegment> walls)
{
    int rescale = 1000;
    cv::Mat base(rescale, rescale, CV_64FC3);
    base=cv::Scalar(255, 255, 255);
    //walls
    for (int i = 0; i < walls.size(); ++i) {
        cv::Point2d p1(walls.at(i).start.x * rescale, 1000 - walls.at(i).start.y * rescale);
        cv::Point2d p2(walls.at(i).end.x * rescale, 1000 - walls.at(i).end.y * rescale);
        cv::line(base, p1, p2, CV_RGB(0, 0, 0), 5, CV_AA);
    }

    // robot
    cv::circle(base, cv::Point(_robot.position.x * rescale, 1000 - _robot.position.y * rescale), _robot.radius * rescale, CV_RGB(200, 0, 0), -1, CV_AA);
    // robot orientation
    cv::Point2d robotCenter(_robot.position.x *rescale,  1000 - _robot.position.y * rescale);
    point orientation; orientation = _robot.position.pointToDis(_robot.radius);
    cv::Point2d robotOr(orientation.x * rescale, 1000 - orientation.y * rescale);

    cv::line(base, robotCenter, robotOr, CV_RGB(0, 0, 0), 5, CV_AA);
    // goal
    cv::circle(base, cv::Point(goal.x * rescale, 1000 - goal.y * rescale), 20, CV_RGB(0, 0, 200), -1, CV_AA);

    cv::namedWindow("maze", CV_NORMAL);
    cv::imshow("maze", base);
    cv::waitKey(10);
}

void RobotMaze::readMaze(robot &_robot, point &goal, vector<lineSegment> &walls)
{
    string line;
    ifstream myfile ("maze.txt");
    if (myfile.is_open())
    {
        while ( getline (myfile,line) )
        {
            std::vector<std::string> tokens;
            boost::split(tokens, line, boost::is_any_of(" "));
            if(tokens[0] == "r"){
                _robot.position.x = atof(tokens[1].c_str());
                _robot.position.y = atof(tokens[2].c_str());
                _robot.position.theta = atof(tokens[3].c_str());
                _robot.radius = atof(tokens[4].c_str());
            }else if(tokens[0] == "g"){
                goal.x = atof(tokens[1].c_str());
                goal.y = atof(tokens[2].c_str());
            }else if(tokens[0] == "w"){
                lineSegment tmp;
                tmp.start.x = atof(tokens[1].c_str());
                tmp.start.y = atof(tokens[2].c_str());
                tmp.end.x = atof(tokens[3].c_str());
                tmp.end.y = atof(tokens[4].c_str());
                walls.push_back(tmp);
            }
        }
        myfile.close();
    }
    else
    {
        cerr << "nofile" << endl;
    }
}

void RobotMaze::processIndividualPostHoc(shared_ptr<NEAT::GeneticIndividual> individual)
{

}

Experiment* RobotMaze::clone()
{
    RobotMaze* experiment = new RobotMaze(*this);

    return experiment;
}
}
