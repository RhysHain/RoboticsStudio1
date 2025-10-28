#include "quadcopter.h"

using namespace controller;

Quadcopter::Quadcopter()    
{
    type_ = data::PlatformType::QUADCOPTER; //Type is quadcopter

    thread_ = new std::thread(&Quadcopter::reachGoal, this);    

}

Quadcopter::~Quadcopter(){
    // Stop the quadcopter immediately, sending it down
    if (thread_->joinable()) {
        thread_->join();
    }
    sendCmd(0, 0, -1.0,0, false); 
}


bool Quadcopter::checkOriginToDestination(data::nav_msgs::Odometry origin, data::geometry_msgs::Point goal,
    double& distance, double& time,
    data::nav_msgs::Odometry& estimatedGoalPose) {
    
    //Finds the distance to the goal
    distance = sqrt(((goal.x - origin.position.x)*(goal.x - origin.position.x))+ ((goal.y - origin.position.y)*(goal.y - origin.position.y)));
    
    //Stored into estimatedGoalPose
    estimatedGoalPose.position.x = goal.x;
    estimatedGoalPose.position.y = goal.y;
    estimatedGoalPose.position.z = goal.z;
    
    //Finds the angle to the goal
    estimatedGoalPose.yaw = std::atan2((goal.y - origin.position.y), (goal.x - origin.position.x));

    //Calculates the steering required to face the goal
    double steering = 0;
    double turn;
    this->computerSteering(origin, goal, steering, estimatedGoalPose.yaw, turn);
    time = distance + std::abs(steering);
    return true;
}

bool Quadcopter::computerSteering(data::nav_msgs::Odometry odo, data::geometry_msgs::Point goal, double& steering, double goalAngle, double& turn) {
    //Finds the steering angle
    steering = goalAngle - odo.yaw;
  
    //Normalises the angle
    if (steering >= M_PI) {
        steering -= 2*M_PI;
    }
    else if (steering <= -1*M_PI) {
        steering += 2*M_PI;
    }
    
    //Converts the angle into a steering direction
    if (steering > 0) { //positive steering means positive turn
        turn = 1;
    }
    else if (steering < 0) { //negative steering means negative turn
        turn = -1;
    }
    else {
        turn = 0.0; //just in case a steering angle of 0 is passed in, as this means I want to stop steering
    }
    return true;
}

void Quadcopter::sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b, bool takeoff) {
    cmd_.move_f_b = move_f_b;
    cmd_.move_u_d = move_u_d;
    cmd_.move_l_r = move_l_r;
    cmd_.turn_l_r = turn_l_r;
    cmd_.takeoff = takeoff;
}

void Quadcopter::reachGoal(void) {
    while(1) {
        while (execute_) {
            std::unique_lock<std::mutex> lck(mtx);
            this->updateTravelData();
            status_ = data::PlatformStatus::RUNNING;
            bool hasTakenOff = false;
            int count = 0;
            //Launch the quadcopter
            while (!hasTakenOff) {
                this->sendCmd(0,0,0,0,true);
                state_ = LAUNCHING;
                this->sendCmd(0,0,this->getToHeight(20), 0, false);
                count++;
                if ((this->getOdometry().position.z > 2) || (count >= 50)) {
                    hasTakenOff = true;
                    state_ = HOVER;
                }
            }
            lck.unlock();
            //Move to each goal
            this->move(goals_);
            lck.lock();
            //Platform set to idle once all goals reached
            execute_ = false;
            this->sendCmd(0,0,0,0,false);
            status_ = data::PlatformStatus::IDLE;
            lck.unlock();
        }
    }
}

double Quadcopter::getToHeight(int height) {
    data::nav_msgs::Odometry odo = this->getOdometry();
    double up;
    if (odo.position.z < height - 2) {
        up = 1;
    }
    else if (odo.position.z > height + 2) {
        up = -1;
    }
    else {
        up = 0;
    }
    return up;
}

bool Quadcopter::move(GoalStats goal) {
    bool goalReached=false;// Indicates if we reach the goal to stop the control
    double steering = 0.0; // Used to store steering to goal
    data::nav_msgs::Odometry estimatedGoalPose; // Store estimated goal position
    double turn;
    double forward;
    //loop reapeats until goal reached
    while(!goalReached) {
        //get current odometry
        std::unique_lock<std::mutex> lck(mtx);
        odo_ = this->getOdometry();
        //calculate distance to goal, estimated goal position and steering value
        this->checkOriginToDestination(odo_, goal.location, goal.distance, goal.time, estimatedGoalPose);
        this->computerSteering(odo_, goal.location, steering, estimatedGoalPose.yaw, turn);
        switch(state_)
        {
            case HOVER: //check if movement is required
                if(goal.distance > tolerance_){
                    state_ = TURNING; //if not at the goal, start the movement process
                }    
                break;
            case TURNING: //turn the car
            //once facing the right direction within a small margin, start moving forward. Otherwise, continue turning
                if ((odo_.yaw < (estimatedGoalPose.yaw + 0.1)) && (odo_.yaw > (estimatedGoalPose.yaw - 0.1))) {
                    state_ = CRUISING;
                    turn = 0.0;
                }
                forward = 0.0; //no moving while doing initial turn
                break;
            case CRUISING: //move forward
            //once within range of the goal, start the breaking state
                if(goal.distance<= 1.1*tolerance_){ 
                    state_ = BREAKING;
                    forward = 0.0;
                }
                else{ //Otherwise move forward and make minor steering adjustments
                    forward = 1.0;
                }
                break;
            case BREAKING: //breaking state
            //if the vehicle has stopped, end the while loop
                if ((odo_.linear.x < 0.01) && (odo_.linear.y < 0.01)){ 
                    goalReached=true;
                }
                //stopp all turning and movement
                forward = 0.0;
                turn = 0.0;
            default:
                break;
        }
        this->updateTravelData();
        this->sendCmd(turn, 0, this->getToHeight(10), forward, false);
        lck.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));//Small delay to ensure message sent
    }
    state_ = MovementState::HOVER;
    return true;
}

data::commands::Quadcopter Quadcopter::getCommands() {
    return cmd_;
}