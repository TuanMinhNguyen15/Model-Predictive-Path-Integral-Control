#define _USE_MATH_DEFINES
#include <iostream>
#include <array>
#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>
#include <functional>
#include "Random.hpp"
#include "MPPI.hpp"

class AugmentedModel : public Model<4,3>{
    private:
        float L;

        // Kinematic Bicycle Model 
        std::array<float,3> KBM(std::array<float,3> state, std::array<float,2> input){
            std::array<float,3> state_dot;
            state_dot[0] = input[0]*std::cos(state[2]);
            state_dot[1] = input[0]*std::sin(state[2]);
            state_dot[2] = input[0]*std::tan(input[1])/(this->L);
            return state_dot;
        }


    public:
        const float Ts = 0.01;

        AugmentedModel(float L){this->L = L;}

        std::array<float,4> Next_State(std::array<float,4> augmented_state, std::array<float,3> augmented_input) override{
            std::array<float,4> next_augmented_state;
            std::array<float,3> state,state_dot,state_next;
            std::array<float,2> input;
            float s,s_dot,s_next;
            float v;

            // Kinematic Bicycle Model
            std::copy(augmented_state.begin(),augmented_state.begin()+3,state.begin());
            std::copy(augmented_input.begin(),augmented_input.begin()+2,input.begin());
            state_dot = KBM(state,input);

            // Path Parametrization Model
            s = augmented_state[3];
            v = augmented_input[2];
            s_dot = v;

            // Computing the next augmented state
            next_augmented_state[0] = state[0] + Ts*state_dot[0];
            next_augmented_state[1] = state[1] + Ts*state_dot[1];
            next_augmented_state[2] = state[2] + Ts*state_dot[2];
            next_augmented_state[3] = s        + Ts*s_dot;
            
            return next_augmented_state;
        }

};

int main(){
    // Setup
    const int N = 50;
    const int K = 200;

    float lag_weight = 10;
    float contour_weight = 10; 
    float progress_weight = 2;

    std::pair<float,float> speed_lim = {0,1};
    std::pair<float,float> steer_lim = {-1,1};
    std::pair<float,float> v_lim = {0,1};

    std::function<std::array<float,3>(float)> path_s = [](float s)->std::array<float,3>{return std::array<float,3> {s,0,0};};

    std::function<float(std::array<float,4>)> tracking_cost = [path_s,lag_weight,contour_weight](std::array<float,4> augumented_state)->float{
        std::array<float,3> state_ref = path_s(augumented_state[3]);
        float el =  std::cos(state_ref[2])*(augumented_state[0]-state_ref[0]) + std::sin(state_ref[2])*(augumented_state[1]-state_ref[1]);
        float ec = -std::sin(state_ref[2])*(augumented_state[0]-state_ref[0]) + std::cos(state_ref[2])*(augumented_state[1]-state_ref[1]);
        return lag_weight*std::abs(el) + contour_weight*std::abs(ec);
    };

    std::function<float(std::array<float,3>)> progress_cost = [progress_weight](std::array<float,3> augumented_input)->float{
        return -progress_weight*augumented_input[2];
    };

    AugmentedModel mobile_robot(0.2);
    MPPI<4,3,N,K> controller(&mobile_robot,{speed_lim,steer_lim,v_lim});
    controller.Set_Lambda(100); // 100
    controller.AddStateCost(std::make_pair(tracking_cost,std::make_pair(0,N-1)));
    controller.AddInputCost(std::make_pair(progress_cost,std::make_pair(0,N-1)));

    
    std::array<float,4> augmented_state_0 = {0,1,0,0};
    std::vector<float> T_sim = {0};
    std::vector<std::array<float,4>> augmented_state_sim = {augmented_state_0};
    std::vector<std::array<float,3>> augmented_input_sim;
    float T_end = 5;

    // Warm up
    controller.UpdateWidthPercent(0.7);
    controller.Run(augmented_state_0,1000);


    // Simulation
    std::array<float,3> augmented_input;
    controller.UpdateWidthPercent(0.2);
    while(T_sim.back() <= T_end){
        augmented_input = controller.Run(augmented_state_sim.back(),50); //70
        augmented_state_sim.push_back(mobile_robot.Next_State(augmented_state_sim.back(),augmented_input));
        augmented_input_sim.push_back(augmented_input);
        T_sim.push_back(T_sim.back()+mobile_robot.Ts);
    }
    augmented_input_sim.push_back(augmented_input);

    // Logging results
    FILE* x_vs_y;
    FILE* velocity_vs_time;
    FILE* steering_vs_time;
    x_vs_y = fopen("x_vs_y.tmp","w");
    velocity_vs_time = fopen("velocity_vs_time.tmp","w");
    steering_vs_time = fopen("steering_vs_time.tmp","w");
    for (int i = 0; i < T_sim.size(); i++){
        fprintf(x_vs_y, "%f %f\n",augmented_state_sim[i][0] ,augmented_state_sim[i][1]);
        fprintf(velocity_vs_time, "%f %f\n",T_sim[i] ,augmented_input_sim[i][0]);
        fprintf(steering_vs_time, "%f %f\n",T_sim[i] ,augmented_input_sim[i][1]);
    }
    fclose(x_vs_y);
    fclose(velocity_vs_time);
    fclose(steering_vs_time);

    return 0;
}