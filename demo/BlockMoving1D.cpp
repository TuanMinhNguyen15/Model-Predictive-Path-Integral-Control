#include <iostream>
#include <array>
#include <vector>
#include <cmath>
#include "MPPI.hpp"

class Block1D : public Model<2,1>{
    private:
        float mass;

        std::array<float,2> dynamics(std::array<float,2> x, std::array<float,1> u){
            std::array<float,2> x_dot;
            x_dot[0] = x[1];
            x_dot[1] = u[0]/(this->mass);

            return x_dot;
        }

    public:
        const float Ts = 0.1;
        Block1D(float mass):Model(){
            this->mass = mass;
        }

        std::array<float,2> Next_State(std::array<float,2> x, std::array<float,1> u) override {
            std::array<float,2> x_next;
            std::array<float,2> x_dot = dynamics(x,u);
            
            x_next[0] = x[0] + Ts*x_dot[0];
            x_next[1] = x[1] + Ts*x_dot[1];

            return x_next;
        }


};

int main(){
    const int N = 20;
    const int K = 50;

    std::array<float,2> x_ref = {1,0};

    float position_cost_coef = 4;
    float velocity_cost_coef = 1;
    float terminal_position_cost_coef = 5;
    float terminal_velocity_cost_coef = 3;

    std::pair<float,float> ulim = {-1,1};
    Block1D block(1);   
    MPPI<2,1,N,K> controller(&block,{ulim});
    controller.AddStateCost(std::make_pair([position_cost_coef,velocity_cost_coef,x_ref](std::array<float,2> x)->float{return std::pow(x[0]-x_ref[0],2)*position_cost_coef + std::pow(x[1]-x_ref[1],2)*velocity_cost_coef;},std::make_pair(0,N-1)));
    controller.AddStateCost(std::make_pair([terminal_position_cost_coef,terminal_velocity_cost_coef,x_ref](std::array<float,2> x)->float{return std::pow(x[0]-x_ref[0],2)*terminal_position_cost_coef + std::pow(x[1]-x_ref[1],2)*terminal_velocity_cost_coef;},std::make_pair(N,N)));
    controller.Set_Lambda(10);

    std::array<float,2> x0 = {0,0};
    std::vector<float> T_sim = {0};
    std::vector<std::array<float,2>> x_sim = {x0};
    std::vector<std::array<float,1>> u_sim;
    float T_end = 10;

    // Warm up
    controller.UpdateWidthPercent(0.7);
    controller.Run(x0,200); 

    // Simulation
    controller.UpdateWidthPercent(0.2);
    std::array<float,1> u;
    while(T_sim.back() <= T_end){
        u = controller.Run(x_sim.back(),50);
        x_sim.push_back(block.Next_State(x_sim.back(),u));
        u_sim.push_back(u);
        T_sim.push_back(T_sim.back()+block.Ts);
    }
    u_sim.push_back(u);

    // // Logging results
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