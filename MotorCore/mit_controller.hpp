#ifndef MOTOR_CORE_MIT_CONTROLLER_HPP_
#define MOTOR_CORE_MIT_CONTROLLER_HPP_

namespace mit_controller
{

struct Command
{
    float p_des;
    float v_des;
    float kp;
    float kd;
    float tau_ff;
};

struct Feedback
{
    float p_now;
    float v_now;
};

struct Limit
{
    float tau_limit;
    float pos_err_limit;
    float vel_err_limit;
};

struct Output
{
    float pos_err;
    float vel_err;
    float tau_cmd;
};

class Controller
{
 public:
    Output output;

    explicit Controller(const Limit &limit);

    void setLimit(const Limit &limit);
    void setEnable(bool enable);
    void setCommand(const Command &command);
    void setFeedback(const Feedback &feedback);
    Output update();

 private:
    Command command_;
    Feedback feedback_;
    Limit limit_;
    bool enable_;

    float clamp(float value, float min_value, float max_value) const;
    float absolute(float value) const;
};

}  // namespace mit_controller

namespace dm_mit = mit_controller;

#endif
