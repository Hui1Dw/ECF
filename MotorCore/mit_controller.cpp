#include "MotorCore/mit_controller.hpp"

namespace mit_controller
{

Controller::Controller(const Limit &limit)
    : output{0.0F, 0.0F, 0.0F},
      command_{0.0F, 0.0F, 0.0F, 0.0F, 0.0F},
      feedback_{0.0F, 0.0F},
      limit_{0.0F, 0.0F, 0.0F},
      enable_(false)
{
    setLimit(limit);
}

void Controller::setLimit(const Limit &limit)
{
    limit_.tau_limit = absolute(limit.tau_limit);
    limit_.pos_err_limit = absolute(limit.pos_err_limit);
    limit_.vel_err_limit = absolute(limit.vel_err_limit);
}

void Controller::setEnable(bool enable)
{
    enable_ = enable;
}

void Controller::setCommand(const Command &command)
{
    command_ = command;
}

void Controller::setFeedback(const Feedback &feedback)
{
    feedback_ = feedback;
}

Output Controller::update()
{
    output = {0.0F, 0.0F, 0.0F};
    if (!enable_)
    {
        return output;
    }

    output.pos_err = clamp(command_.p_des - feedback_.p_now,
                           -limit_.pos_err_limit,
                           limit_.pos_err_limit);
    output.vel_err = clamp(command_.v_des - feedback_.v_now,
                           -limit_.vel_err_limit,
                           limit_.vel_err_limit);

    const float raw_tau_cmd = command_.kp * output.pos_err
                            + command_.kd * output.vel_err
                            + command_.tau_ff;
    output.tau_cmd = clamp(raw_tau_cmd,
                           -limit_.tau_limit,
                           limit_.tau_limit);
    return output;
}

float Controller::clamp(float value, float min_value, float max_value) const
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

float Controller::absolute(float value) const
{
    return value < 0.0F ? -value : value;
}

}  // namespace mit_controller
