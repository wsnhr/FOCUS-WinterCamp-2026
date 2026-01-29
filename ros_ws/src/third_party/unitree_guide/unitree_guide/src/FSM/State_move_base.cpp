/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_MOVE_BASE

#include "FSM/State_move_base.h"

State_move_base::State_move_base(CtrlComponents *ctrlComp)
    :State_Trotting(ctrlComp){
    _vx = 0.0;
    _vy = 0.0;
    _wz = 0.0;
    _hasCmd = false;
    _cmdTimeout = 0.3;           // 0.3秒没收到cmd_vel就刹车（你也可以改 0.1~0.5
    _stateName = FSMStateName::MOVE_BASE;
    _stateNameString = "move_base";
    initRecv();
}

FSMStateName State_move_base::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::MOVE_BASE;
    }
}

void State_move_base::getUserCmd(){
	ros::spinOnce();
// watchdog：一段时间没收到cmd_vel，就自动停
    if(!_hasCmd || (ros::Time::now() - _lastCmdTime).toSec() > _cmdTimeout){
        _vx = 0.0;
        _vy = 0.0;
        _wz = 0.0;
    }

    
    
    setHighCmd(_vx, _vy, _wz);
    
}

void State_move_base::twistCallback(const geometry_msgs::Twist& msg){
    _vx = msg.linear.x;
    _vy = msg.linear.y;
    _wz = msg.angular.z;
    _lastCmdTime = ros::Time::now();
    _hasCmd = true;
}

void State_move_base::initRecv(){
    _cmdSub = _nm.subscribe("/cmd_vel", 1, &State_move_base::twistCallback, this);
}

#endif  // COMPILE_WITH_MOVE_BASE
