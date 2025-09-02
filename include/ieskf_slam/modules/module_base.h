/*
 * @Descripttion:
 * @Author: MengKai
 * @version:
 * @Date: 2023-06-08 23:59:37
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-10 01:21:13
 */
#pragma once
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <string>

#include "colorful_terminal/colorful_terminal.hpp"
namespace IESKFSlam {
    // 用于读取配置文件的基类
    class ModuleBase {
       private:
        YAML::Node config_node;
        std::string name;
        std::shared_ptr<ctl::table_out> table_out_ptr;

       protected:
        /**
         * @brief 构造函数
         * @param config_path: 配置文件目录
         * @param prefix: 前缀
         * @param module_name: 模块名称
         */
        // 如果不希望基类被实例化，可以将基类的构造函数声明为protected
        ModuleBase(const std::string &config_path, const std::string &prefix,
                   const std::string &module_name = "default") {
            name = module_name;
            // colorful terminal print out
            table_out_ptr = std::make_shared<ctl::table_out>(module_name);
            if (config_path != "") {
                try {
                    // yaml-cpp 读取配置文件的方法
                    config_node = YAML::LoadFile(config_path);
                } catch (YAML::Exception &e) {
                    std::cout << e.msg << std::endl;
                }

                if (prefix != "" && config_node[prefix]) config_node = config_node[prefix];
            }
        }
        /**
         * @param T
         * @param key: 键值
         * @param val: 读取数据到哪个参数
         * @param default_val: 默认值
         * @brief 读取配置文件参数，仿照着ros的param写的
         */
        template <typename T>
        void readParam(const std::string &key, T &val, T default_val) {
            if (config_node[key]) {
                // as<T>() 是yaml-cpp 提供的将节点转换为指定类型的方法
                val = config_node[key].as<T>();
            } else {
                val = default_val;
            }
            // colorful terminal add var
            table_out_ptr->add_item(key, VAR_NAME(val), val);
            // std::cout<<name: <<default_val<<std::endl;
        }
        // 方便输出参数查看，仅限于子类调用
        void print_table() { table_out_ptr->make_table_and_out(); }
    };
}  // namespace IESKFSlam
