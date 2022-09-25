#include "MPPI.hpp"

// MPPI
template <std::size_t n, std::size_t m, int N, int K>
std::array<std::array<float,n>,N+1> MPPI<n,m,N,K>::Simulate(std::array<float,n> x0, std::array<std::array<float,m>,N> u_seq){
    std::array<std::array<float,n>,N+1> x_seq;

    x_seq[0] = x0;
    for (int i = 1; i <= N ; i++){
        x_seq[i] = this->model->Next_State(x_seq[i-1],u_seq[i-1]);
    }

    return x_seq;
}

template <std::size_t n, std::size_t m, int N, int K>
float MPPI<n,m,N,K>::Evaluate(std::array<std::array<float,n>,N+1> x_seq, std::array<std::array<float,m>,N> u_seq){
    float total_cost = 0;
    for (int i = 0; i <= N; i++){
        // Add State Costs
        for (auto state_cost : this->state_costs){
            if (i <= (state_cost.second).second && i >= (state_cost.second).first){
                total_cost += (state_cost.first)(x_seq[i]);
            }
        }
     
        // Add Input Costs
        for (auto input_cost : this->input_costs){
            if (i <= (input_cost.second).second && i >= (input_cost.second).first){
                total_cost += (input_cost.first)(u_seq[i]);
            }
        }
        
    }

    return total_cost;
}

template <std::size_t n, std::size_t m, int N, int K>
void  MPPI<n,m,N,K>::Trajectory_RollOut(std::array<float,n> x0, std::array<std::array<float,m>,N> u_seq, int index_candidate){
    this->costs_candidates[index_candidate] = Evaluate(Simulate(x0, u_seq),u_seq);
    this->u_seq_candidates[index_candidate] = u_seq;
}


template <std::size_t n, std::size_t m, int N, int K>
std::array<std::array<float,m>,N> MPPI<n,m,N,K>::Sampling_u_seq(std::array<std::array<float,m>,N> u_seq_mode){
    std::array<std::array<float,m>,N> u_seq_sampled;
    float sampling_width;
    float lb,ub;

    for (int index_timestep = 0; index_timestep < N; index_timestep++){
        for (int index_input = 0; index_input < m; index_input++){
            sampling_width = (this->u_lim[index_input].second - this->u_lim[index_input].first)*(this->width_percent);   
            ub = u_seq_mode[index_timestep][index_input] + sampling_width/2.;
            lb = u_seq_mode[index_timestep][index_input] - sampling_width/2.;

            if (ub > this->u_lim[index_input].second){
                ub = this->u_lim[index_input].second;
                lb = this->u_lim[index_input].second - sampling_width;
            }

            if (lb < this->u_lim[index_input].first){
                lb = this->u_lim[index_input].first;
                ub = this->u_lim[index_input].first + sampling_width;
            }

            Uniform_Distr::Update_Param(lb,ub);
            u_seq_sampled[index_timestep][index_input] = Uniform_Distr::Gen();
        }
    }

    return u_seq_sampled;
}


template <std::size_t n, std::size_t m, int N, int K>
MPPI<n,m,N,K>::MPPI(Model<n,m> *model, std::array<std::pair<float,float>,m> u_lim){
    this->model = model;
    this->u_lim = u_lim;
    std::array<float,m> u;
    
    for (int i = 0 ; i < N ; i++){
        for (int j = 0 ; j < m ; j++){
            this->u_seq_optimized[i][j] = (u_lim[j].first + u_lim[j].second)*0.5;
        }
        this->u_seq_explored = this->u_seq_optimized;
    }

}

template <std::size_t n, std::size_t m, int N, int K>
void MPPI<n,m,N,K>::AddModel(Model<n,m> *model){
    this->model = model;
}

template <std::size_t n, std::size_t m, int N, int K>
void MPPI<n,m,N,K>::AddStateCost(std::pair<std::function<float(std::array<float,n>)>,std::pair<int,int>> state_cost){
    this->state_costs.push_back(state_cost);
}

template <std::size_t n, std::size_t m, int N, int K>
void MPPI<n,m,N,K>::AddInputCost(std::pair<std::function<float(std::array<float,m>)>,std::pair<int,int>> input_cost){
    this->input_costs.push_back(input_cost);
}

template <std::size_t n, std::size_t m, int N, int K>
void MPPI<n,m,N,K>::UpdateStateCost(std::pair<std::function<float(std::array<float,n>)>,std::pair<int,int>> state_cost, uint8_t index){
    this->state_costs[index] = state_cost;
}

template <std::size_t n, std::size_t m, int N, int K>
void MPPI<n,m,N,K>::UpdateInputCost(std::pair<std::function<float(std::array<float,m>)>,std::pair<int,int>> input_cost, uint8_t index){
    this->input_costs[index] = input_cost;
}

template <std::size_t n, std::size_t m, int N, int K>
void MPPI<n,m,N,K>::Set_Lambda(float lambda){
    this->lambda = lambda;
}


template <std::size_t n, std::size_t m, int N, int K>
std::array<std::array<float,m>,N> MPPI<n,m,N,K>::Test_Run(std::array<float,n> x0){
    float e_total_cost, e_cost_k;
    float cost_current, cost_next;
    float u;
    std::array<std::array<float,m>,N> u_seq_out; 
    

    // Evaluate input candidates
    for (int index_candidate = 0; index_candidate < K; index_candidate++){   
        this->threads[index_candidate] = std::thread([this](std::array<float,n> _x0, std::array<std::array<float,m>,N> u_seq, int index)->void{this->Trajectory_RollOut(_x0,u_seq,index);},
                                                        x0,Sampling_u_seq(this->u_seq_explored),index_candidate);
    }

    for (std::thread &thread : this->threads){
        thread.join();
    }


    // Computing new optimized input
    e_total_cost = 0.;
    for (float cost : this->costs_candidates){
        e_total_cost += std::exp(-cost/this->lambda);
    }
    if (e_total_cost <= 1e-3){
        throw std::runtime_error("Divide by zero due to lambda is too small!!!");
    }
    
    for (int index_timestep = 0; index_timestep < N; index_timestep++){
        for (int index_input = 0; index_input < m; index_input++){
            u = this->u_seq_explored[index_timestep][index_input]; 
            for (int index_candidate = 0; index_candidate < K; index_candidate++){  
                e_cost_k = std::exp(-(this->costs_candidates[index_candidate])/this->lambda);
                u += (e_cost_k/e_total_cost)*(this->u_seq_candidates[index_candidate][index_timestep][index_input] - this->u_seq_explored[index_timestep][index_input]);  // here
            }

            u_seq_out[index_timestep][index_input] = u;
        }
    }

    return u_seq_out;
    
}


template <std::size_t n, std::size_t m, int N, int K>
std::array<float,m> MPPI<n,m,N,K>::Run(std::array<float,n> x0, int trial_max){
    float cost_new, cost_best;
    std::array<std::array<float,m>,N> u_seq_new;
    int trial_num = 0;

    cost_best = Evaluate(Simulate(x0,this->u_seq_optimized),this->u_seq_optimized);

    this->u_seq_explored = this->u_seq_optimized;
    while(trial_num < trial_max){
        try{
            u_seq_new = Test_Run(x0);
        }
        catch(std::runtime_error &error){
            std::cerr << error.what() << std::endl;
            std::exit(1);
        }

        
        cost_new = Evaluate(Simulate(x0,u_seq_new),u_seq_new);
        if (cost_new < cost_best){
            this->u_seq_optimized = u_seq_new;
            cost_best = cost_new;

        }

        this->u_seq_explored = u_seq_new;
        trial_num++;
    }

    return this->u_seq_optimized[0];
}

template <std::size_t n, std::size_t m, int N, int K>
std::array<std::array<float,m>,N> MPPI<n,m,N,K>::Get_u_seq_optimized(){
    return this->u_seq_optimized;   
}

template <std::size_t n, std::size_t m, int N, int K>
void MPPI<n,m,N,K>::Shift_u_seq_optmized(){
    for (int i = 0; i < N-1; i++){
        this->u_seq_optimized[i] = this->u_seq_optimized[i+1];
    }

    this->u_seq_optimized[N-1] = this->u_seq_optimized[N-2];
}

template <std::size_t n, std::size_t m, int N, int K>
void MPPI<n,m,N,K>::UpdateWidthPercent(float width_percent){
    if (width_percent <= 1 && width_percent >= 0){
        this->width_percent = width_percent;
    }
    else{
        std::cout << "width_percent must be between 0 and 1\n";
    }
}
