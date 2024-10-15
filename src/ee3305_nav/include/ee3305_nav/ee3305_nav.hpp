#include <iostream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"

#pragma once
namespace ee3305 
{
    /**
     * Declares a parameter and assigns any set value (from YAML) to "to". 
     * @param param_name the name of the parameter
     * @param to must be initialized with a default value.
     * @param cout_width the width of the printed string describing the parameter. Set to zero to disable the print.
    */
    template <typename T>
    void initParam(rclcpp::Node *const &node, const std::string &param_name, T &to, const size_t &cout_width=25)
    {   
        if (node->has_parameter(param_name) == false)
            node->declare_parameter<T>(param_name, to);
        node->get_parameter_or<T>(param_name, to, to);
        
        if (cout_width > 0)
            std::cout << std::setw(cout_width) << param_name << ": " << to << std::endl;
    }
}