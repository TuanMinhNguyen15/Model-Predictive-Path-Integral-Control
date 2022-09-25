#ifndef _MPPI_
#define _MPPI_

#include <iostream>
#include <stdio.h>
#include <cmath>
#include <vector>
#include <array>
#include <functional>
#include <utility>
#include <thread>
#include <cmath>
#include <exception>
#include "Random.hpp"

// System Model Base Class
template<std::size_t n, std::size_t m>
class Model{
    public:
        Model(){}
        virtual std::array<float,n> Next_State(std::array<float,n> x0, std::array<float,m> u_seq) = 0;
};

// MPPI Class
template <std::size_t n, std::size_t m, int N, int K = 10>
class MPPI{
    private:
        float width_percent = 0.2;
        float lambda = 1;
        Model<n,m> *model;
        std::vector<std::pair<std::function<float(std::array<float,n>)>,std::pair<int,int>>> state_costs;
        std::vector<std::pair<std::function<float(std::array<float,m>)>,std::pair<int,int>>> input_costs;
        std::array<std::pair<float,float>,m> u_lim;
        std::array<std::array<std::array<float,m>,N>,K+1> u_seq_candidates;
        std::array<float,K+1> costs_candidates;
        std::array<std::thread,K> threads;

        std::array<std::array<float,n>,N+1> Simulate(std::array<float,n> x0, std::array<std::array<float,m>,N> u_seq);
        float Evaluate(std::array<std::array<float,n>,N+1> x_seq, std::array<std::array<float,m>,N> u_seq);
        void Trajectory_RollOut(std::array<float,n> x0, std::array<std::array<float,m>,N> u_seq, int index_candidate);
        std::array<std::array<float,m>,N> Sampling_u_seq(std::array<std::array<float,m>,N> u_seq_mode);
        std::array<std::array<float,m>,N> Test_Run(std::array<float,n> x0);

          
    public:
        MPPI(Model<n,m> *model, std::array<std::pair<float,float>,m> u_lim);
        std::array<std::array<float,m>,N> u_seq_optimized;
        std::array<std::array<float,m>,N> u_seq_explored;
        void AddModel(Model<n,m> *model);
        void AddStateCost(std::pair<std::function<float(std::array<float,n>)>,std::pair<int,int>> state_cost);
        void AddInputCost(std::pair<std::function<float(std::array<float,m>)>,std::pair<int,int>> input_cost);
        void UpdateStateCost(std::pair<std::function<float(std::array<float,n>)>,std::pair<int,int>> state_cost, uint8_t index);
        void UpdateInputCost(std::pair<std::function<float(std::array<float,m>)>,std::pair<int,int>> input_cost, uint8_t index);
        void Set_Lambda(float lambda);
        std::array<float,m> Run(std::array<float,n> x0, int trial_max = 10);
        std::array<std::array<float,m>,N> Get_u_seq_optimized();
        void Shift_u_seq_optmized();
        void UpdateWidthPercent(float width_percent);
};

#include "MPPI.cpp"

#endif

