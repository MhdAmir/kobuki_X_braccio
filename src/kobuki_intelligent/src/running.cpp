#include "kobuki_intelligent/intelligent.h"

void Intelligent::Loop()
{
    switch (mode_communication_)
    {
    case 0:
        // Init();
        break;
    
    case 1:
        AutomaticSearchCup();
        break;

    case 2:
        FollowHuman();
        break;

    case 3:

        break;
    
    case 4:

        break;
    }
    // AutomaticSearchCup();
    ApplyAcceleration();
    PublishMessage();
}

void Intelligent::PublishMessage()
{
    kobuki_msgs::MotorPower motor_msg;
    geometry_msgs::Twist twist_msg;
    std_msgs::UInt8MultiArray angle_msg;

    twist_msg.linear.x = move_fb;
    twist_msg.angular.z = move_rl;

    if (arm_moving_)
    {
        return;
    }

    motor_msg.state = kobuki_msgs::MotorPower::ON;

    if (pub_kobuki_velocity_.getNumSubscribers() > 0 /*&& pub_arm_.getNumSubscribers() > 0*/)
    {
        pub_kobuki_velocity_.publish(twist_msg);
        angle_msg.data = angles_;
        pub_arm_.publish(angle_msg);
        last_angles_ = angles_;
    }
    else
    {
        state_ = 0;
    }

}

void Intelligent::FollowHuman()
{
    const int FRAME_CENTER = 0;
    const int SMALL_DEVIATION_THRESHOLD = 15;
    const int LARGE_DEVIATION_THRESHOLD = 100;

    int deviation = object_[kPerson].frame.x - FRAME_CENTER;
    double rotation_speed;

    double filtered_distance = GetFilteredDistance(object_[kPerson].distance);

    fprintf(stderr, "filter >> %g \n", filtered_distance);

    if(!object_[kPerson].exist){
        target_fb = 0.0;
    }else{
        deviation = object_[kPerson].frame.x;
        if (abs(deviation) > LARGE_DEVIATION_THRESHOLD)
        {
            target_fb = 0.0;
            rotation_speed = 0.3;
            target_rl = (deviation > 0) ? -rotation_speed : rotation_speed;
        }
        else if (abs(deviation) > SMALL_DEVIATION_THRESHOLD)
        {
            target_fb = 0.1;
            rotation_speed = 0.1 + (abs(deviation) - SMALL_DEVIATION_THRESHOLD) * 0.002;
            target_rl = (deviation > 0) ? -rotation_speed : rotation_speed;
        }
        else
        {
            target_fb = 0.2;
            rotation_speed = abs(deviation) * 0.003;
            target_rl = (deviation > 0) ? -rotation_speed : rotation_speed;
        }
    
        if (filtered_distance < 1.2)
        {
            if (abs(deviation) > SMALL_DEVIATION_THRESHOLD)
            {
                rotation_speed = 0.1 + (abs(deviation) - SMALL_DEVIATION_THRESHOLD) * 0.002;
                target_rl = (deviation > 0) ? -rotation_speed : rotation_speed;
            }
            target_fb = 0.0;
            
        }
    }
    
    
}


void Intelligent::AutomaticSearchCup()
{
    const int FRAME_CENTER = 0;
    const int SMALL_DEVIATION_THRESHOLD = 15;
    const int LARGE_DEVIATION_THRESHOLD = 100;

    int deviation = object_[kCup].frame.x - FRAME_CENTER;
    double rotation_speed;

    double filtered_distance = GetFilteredDistance(object_[kCup].distance);

    fprintf(stderr, "filter >> %g \n", filtered_distance);
    
    if(state_ != 2 && state_ != 3 && state_ != 4 && state_ != 5 ){
        if(filtered_distance > 0.3){
            state_ = 0;
        }else if(filtered_distance > 0.2){
            state_ = 1;
        }

    }

    // fprintf(stderr, "state >> %d\n", state_);
    // fprintf(stderr, "arm move >> %d\n\n", arm_moving_);
    // fprintf(stderr, "frame x >> %d\n\n", object_[kCup].frame.x);

    switch (state_)
    {
    case 0:
        if (!object_[kCup].exist)
        {
            angles_.assign({151, 83, 0, 0, 90, 90});
            target_fb = 0.0;
            target_rl = 0.3;
        }
        else
        {
            angles_.assign({90, 120, 0, 0, 90, 90});
            deviation = object_[kCup].frame.x;
            if (abs(deviation) > LARGE_DEVIATION_THRESHOLD)
            {
                target_fb = 0.0;
                rotation_speed = 0.3;
                target_rl = (deviation > 0) ? -rotation_speed : rotation_speed;
            }
            else if (abs(deviation) > SMALL_DEVIATION_THRESHOLD)
            {
                target_fb = 0.1;
                rotation_speed = 0.1 + (abs(deviation) - SMALL_DEVIATION_THRESHOLD) * 0.002;
                target_rl = (deviation > 0) ? -rotation_speed : rotation_speed;
            }
            else
            {
                target_fb = 0.2;
                rotation_speed = abs(deviation) * 0.003;
                target_rl = (deviation > 0) ? -rotation_speed : rotation_speed;
            }

            if (filtered_distance < 0.3)
            {
                if (abs(deviation) > SMALL_DEVIATION_THRESHOLD)
                {
                    rotation_speed = 0.1 + (abs(deviation) - SMALL_DEVIATION_THRESHOLD) * 0.002;
                    target_rl = (deviation > 0) ? -rotation_speed : rotation_speed;
                }
                target_fb = 0.0;
                
                if(abs(object_[kCup].frame.x) > SMALL_DEVIATION_THRESHOLD)
                    target_rl = 0.0;
                    state_ = 1;
            }
        }
        break;
    case 1:
        angles_.assign({92, 73, 0, 88, 90, 0});

        deviation = object_[kCup].frame.x;

        if (abs(deviation) > SMALL_DEVIATION_THRESHOLD)
        {
            target_fb = 0.0;
            rotation_speed = 0.2;
            target_rl = (deviation > 0) ? -rotation_speed : rotation_speed;
        }
        else
        {
            target_fb = 0.1;
            rotation_speed = abs(deviation) * 0.003;
            target_rl = (deviation > 0) ? -rotation_speed : rotation_speed;
            if(filtered_distance < 0.15){
                target_fb = -0.3;
            }
            else if(!arm_moving_ && filtered_distance < 0.2){
                
                if(abs(object_[kCup].frame.x) < SMALL_DEVIATION_THRESHOLD){
                    state_ = 2;
                    target_rl = 0;
                }
                target_fb = 0;
            }
        }
        
        break;
        
    case 2:     
        angles_.assign({90, 0, 0, 145, 93, 0});
        if(!arm_moving_){
            state_ = 3;
            target_fb = 0;
            target_rl = 0;
        }
        // target_fb = 0;
        break;

    case 3:
        angles_.assign({90, 0, 0, 145, 93, 180});
        if(!arm_moving_){
            state_ = 4;
            target_fb = 0;
            target_rl = 0;
        }
        // target_fb = 0;
        break;
        
    case 4:
        angles_.assign({90, 90, 180, 145, 93, 180});
        if(!arm_moving_){
            state_ = 5;
            target_fb = 0;
            target_rl = 0;
        }
        // target_fb = 0;
        break;

    case 5:
        angles_.assign({90, 90, 180, 145, 93, 0});
        if(!arm_moving_){
            state_ = 0;
            target_fb = 0;
            target_rl = 0;
        }
        // target_fb = 0;
        break;
    }
}

void Intelligent::ApplyAcceleration()
{
    double accel_fb = 0.02;
    double accel_rl = 0.02;

    if (move_fb < target_fb)
        move_fb = std::min(move_fb + accel_fb, target_fb);
    else if (move_fb > target_fb)
        move_fb = std::max(move_fb - accel_fb, target_fb);

    if (move_rl < target_rl)
        move_rl = std::min(move_rl + accel_rl, target_rl);
    else if (move_rl > target_rl)
        move_rl = std::max(move_rl - accel_rl, target_rl);
}

double Intelligent::GetFilteredDistance(double new_distance) {
    if(new_distance > 0){
        if (distance_history_.size() >= HISTORY_SIZE) {
            distance_history_.erase(distance_history_.begin()); 
        }
        distance_history_.push_back(new_distance);
    
    }
    double sum = 0;
    int count = 0;
    for (double d : distance_history_) {
        if (d > 0) { 
            sum += d;
            count++;
        }
    }

    return (count > 0) ? (sum / count) : 0;
}