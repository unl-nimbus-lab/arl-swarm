#include "custom_libs/id_to_idx.hpp"

int id_to_idx::get_state_idx(int id, int *groups){
        // This function returns the array index of the states w.r.t the agent group
        int n_dim = 2 ;
        int n_states = 2 ;

        // Index of the state array where an agent should start writing its states
        int idx = 0 ;

        // size of the groups array
        int n_groups = sizeof(groups)/sizeof(groups[0]);

        int members_before_this_group = 0 ; // Accumulates the agent count in each group
        int members_including_this_group = 0 ;

         // TODO : Make sure the agents are IDs from 1 .. N .. if IDed from 0 ... N, change id-1 to id
        for(int i=0 ; i<n_groups ; i++){

                members_including_this_group = members_including_this_group + groups[i];
                members_before_this_group = members_including_this_group - groups[i] ;

                if(members_including_this_group >= id){
                        // each agent in a group has 6 states. (pos+vel in x,y directions + 2 integral states)
                        int idx_for_past_memebrs = members_before_this_group*(n_dim*n_states + n_states) ;
                        int idx_within_group = (id - members_before_this_group - 1)*(n_dim*n_states) ;
                        idx = idx_for_past_memebrs + idx_within_group ;
                        break ;
                }
        }

        return idx ;
}

int id_to_idx::get_integral_idx(int id, int *groups){

        // This function returns the array index of the integral states w.r.t the agent group
        int n_dim = 2 ;
        int n_states = 2 ;

        // Index of the state array where an agent should start writing its states
        int idx = 0 ;

        // size of the groups array
        int n_groups = sizeof(groups)/sizeof(groups[0]);

        int members_before_this_group = 0 ; // Accumulates the agent count in each group
        int members_including_this_group = 0 ;

         // TODO : Make sure the agents are IDs from 1 .. N .. if IDed from 0 ... N, change id-1 to id
        for(int i=0 ; i<n_groups ; i++){

                members_including_this_group = members_including_this_group + groups[i];
                members_before_this_group = members_including_this_group - groups[i] ;

                if(members_including_this_group >= id){
                        // each agent in a group has 6 states. (pos+vel in x,y directions + 2 integral states)
                        int idx_for_past_memebrs = members_before_this_group*(n_dim*n_states + n_states) ;
                        int idx_within_group = groups[i]*(n_dim*n_states) + (id - members_before_this_group - 1)*(n_states) ;
                        idx = idx_for_past_memebrs + idx_within_group ;
                        break ;
                }
        }

        return idx ;
}
