#define _USE_MATH_DEFINES
#include <iostream>
#include <array>
#include <utility>
#include <vector>
#include <cmath>
#include "MPPI.hpp"

class Pendulum : public Model<2,1>{
    private:
        const float g = 9.81;
        float l,m,k;

        std::array<float,2> dynamics(std::array<float,2> x, std::array<float,1> u){
            std::array<float,2> x_dot;
            x_dot[0] = x[1];
            x_dot[1] = (-g/l)*std::sin(x[0]) -(k/m)*x[1] + u[0]/(m*std::pow(l,2));

            return x_dot;
        }



    public:
        const float Ts = 0.01;

        Pendulum(float l, float m, float k):Model(){
            this->l = l;
            this->m = m;
            this->k = k;
        }

        std::array<float,2> Next_State(std::array<float,2> x, std::array<float,1> u) override {
            std::array<float,2> x_dot,x_next;
            
            x_dot = dynamics(x,u);
            x_next[0] = x_dot[0]*Ts + x[0];
            x_next[1] = x_dot[1]*Ts + x[1];

            return x_next;
        }
};


int main(){
    float length  = 1;
    float mass = 1; 
    float damping = 2;
    Pendulum pendulum(length, mass, damping);
    

    const int N = 20;
    const int K = 100;

    float theta_ref = M_PI_4;
    std::array<float,2> x_ref = {theta_ref,0};
    std::pair<float,float> u_lim = {-10,10};

    float theta_cost_coef = 15;
    float thetadot_cost_coef = 1;
    float terminal_theta_cost_coef = 20;
    float terminal_thetadot_cost_coef = 2;



    MPPI<2,1,N,K> controller(&pendulum,{u_lim});
    controller.Set_Lambda(50); 
    controller.AddStateCost(std::make_pair([theta_cost_coef,thetadot_cost_coef,x_ref](std::array<float,2> x)->float{return theta_cost_coef*std::pow(x[0]-x_ref[0],2) + thetadot_cost_coef*std::pow(x[1]-x_ref[1],2);},std::make_pair(0,N-1)));
    controller.AddStateCost(std::make_pair([terminal_theta_cost_coef,terminal_thetadot_cost_coef,x_ref](std::array<float,2> x)->float{return terminal_theta_cost_coef*std::pow(x[0]-x_ref[0],2) + terminal_thetadot_cost_coef*std::pow(x[1]-x_ref[1],2);},std::make_pair(N,N)));


    std::array<float,2> x0 = {0,0};
    std::vector<float> T_sim = {0};
    std::vector<std::array<float,2>> x_sim = {x0};
    std::vector<std::array<float,1>> u_sim;
    float T_end = 10;

    // Warm up
    controller.UpdateWidthPercent(0.7);
    controller.Run(x0,1000); 


    // Simulation
    std::array<float,1> u;
    controller.UpdateWidthPercent(0.2);
    while(T_sim.back() <= T_end){
        u = controller.Run(x_sim.back(),20);
        x_sim.push_back(pendulum.Next_State(x_sim.back(),u));
        u_sim.push_back(u);
        T_sim.push_back(T_sim.back()+pendulum.Ts);
    }
    u_sim.push_back(u);

    // Logging results
    FILE* position_vs_time;
    FILE* velocity_vs_time;
    FILE* input_vs_time;
    position_vs_time = fopen("position_vs_time.tmp","w");
    velocity_vs_time = fopen("velocity_vs_time.tmp","w");
    input_vs_time = fopen("input_vs_time.tmp","w");
    for (int i = 0; i < T_sim.size(); i++){
        fprintf(position_vs_time, "%f %f\n",T_sim[i] ,x_sim[i][0]);
        fprintf(velocity_vs_time, "%f %f\n",T_sim[i] ,x_sim[i][1]);
        fprintf(input_vs_time, "%f %f\n",T_sim[i] ,u_sim[i][0]);
    }
    fclose(position_vs_time);
    fclose(velocity_vs_time);
    fclose(input_vs_time);

    
    return 0;
}