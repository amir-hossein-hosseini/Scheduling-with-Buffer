import cplex
import timeit
from cplex.callbacks import LazyConstraintCallback
from Master_Problem import *
from SP_with_CB import *
from SP_Without_CB import *
from SP_Without_CB_Without_MR import *
from Generate_Adjacent_Job_Subsets import *


class MyLazyConstraintCallback:
    def __init__(self, n, m, q, pt, tt, b_input, b_output, Buffer_Policy,Big_M, Number_of_Thread,SP_Time_Limit, LB0_Makespan, UB0_Makespan, MP_Var_Names):

        self.global_UB = float("inf")
        self.global_LB = LB0_Makespan
        self.best_solution = None
        self.n = n
        self.m = m
        self.q = q
        self.pt = pt
        self.tt = tt
        self.b_input = b_input
        self.b_output = b_output
        self.Big_M = Big_M
        self.LB0_Makespan = LB0_Makespan
        self.UB0_Makespan = UB0_Makespan
        self.Number_of_Thread = Number_of_Thread
        self.SP_Time_limit=SP_Time_Limit
        self.MP_Var_Names = MP_Var_Names
        self.Buffer_Policy=Buffer_Policy

        self.iteration = 0
        self.last_explored_node = None
        self.Aggregated_Time_for_Cuts = 0
        self.Problematic_Case=0


    Number_of_Combinatorial_Cut = 0
    Last_Feasible_Node = []
    Last_Value_of_Node = []


    def lazy_constraint(self, context, Current_MP_Solution, Current_MP_ObjVal, feasibility_cut=True, optimality_cut=True):
        Input_Variables_to_SP = [name for name, value in zip(self.MP_Var_Names, Current_MP_Solution)
                                 if name.startswith("x_") and abs(value - 1.0) < 1e-6]

        Sequence_Reported_by_The_MP = [[0 for _ in range(self.n)] for _ in range(self.m)]

        for j in range(self.n):
            for i in range(self.m):
                for k in range(self.n):
                    if Current_MP_Solution[j * (self.m * self.n) + i * self.n + k] > 0:
                        Sequence_Reported_by_The_MP[i][k] = j + 1

        self.iteration += 1
        print(f"___________________________ Iteration {self.iteration} ___________________________")
        print(f"MP: {Sequence_Reported_by_The_MP}, {Current_MP_ObjVal} ")

        if self.Buffer_Policy==0:                      #In case that all buffer options (local, CB and MR) are acitivated
            [LB_on_SP, UB_on_SP, Gap_SP, SP_Running_Time, _, _, _, _, _, Sequence_Reported_by_SP,
            Current_SP_Solution, _] = SP_with_CB(self.n, self.m, self.q, self.tt, self.pt,
                                              self.b_input, self.b_output, self.Big_M,
                                              Input_Variables_to_SP, self.LB0_Makespan, self.UB0_Makespan, self.Number_of_Thread,self.SP_Time_limit)
            print("All buffer policies are activated: Local+ CB+ MR")
        elif self.Buffer_Policy==1:                      #In case that CB policy are deactivated
             [LB_on_SP, UB_on_SP, Gap_SP, SP_Running_Time, _, _, _, _, _, Sequence_Reported_by_SP,
             Current_SP_Solution, _] = SP_Without_CB(self.n, self.m, self.q, self.tt, self.pt,
                                              self.b_input, self.b_output, self.Big_M,
                                              Input_Variables_to_SP, self.LB0_Makespan,self.UB0_Makespan, self.Number_of_Thread,self.SP_Time_limit)
             print("CB policy is deactivated: Local+ MR")
             
        elif self.Buffer_Policy==2:                    #In case that MR and CB policies are deactivated
            [LB_on_SP, UB_on_SP, Gap_SP, SP_Running_Time, _, _, _, _, _, Sequence_Reported_by_SP,
             Current_SP_Solution, _] = SP_Without_CB_Without_MR(self.n, self.m, self.q, self.tt, self.pt,
                                              self.b_input, self.b_output, self.Big_M,
                                              Input_Variables_to_SP, self.LB0_Makespan,self.UB0_Makespan, self.Number_of_Thread,self.SP_Time_limit)
            print("CB and MR policies are deactivated: Local")
            print(Sequence_Reported_by_SP)


       
        if Sequence_Reported_by_SP is not None and Gap_SP > 0.0 and SP_Running_Time> self.SP_Time_limit+5:
            print("WIERD CASE HAPPENED")
            self.Number_of_Thread2=2
            [LB_on_SP, UB_on_SP, Gap_SP, _, _, _, _, _, _, Sequence_Reported_by_SP,
             Current_SP_Solution, _] = SP_Without_CB_Without_MR(self.n, self.m, self.q, self.tt, self.pt,
                                              self.b_input, self.b_output, self.Big_M,
                                              Input_Variables_to_SP, self.LB0_Makespan,self.UB0_Makespan, self.Number_of_Thread2)
            print("SP results are as follow")
            print("UB=",UB_on_SP,"---","LB=",LB_on_SP,"---","Gap=", Gap_SP)


        if Sequence_Reported_by_SP is None or (self.global_UB-LB_on_SP > 0.05 and UB_on_SP-self.global_UB>0.05  ) :
            print(f"Case 1- Unresolved SP")
            print(f"The node is rejected   ----- ")
            print(f"Feasibility cut on  {Sequence_Reported_by_SP} ")
            self.Problematic_Case+= 1

            cut_lhs, sense, cut_rhs = self.Create_Combinatorial_Cut(Input_Variables_to_SP)
            constraint = cplex.SparsePair(ind=Input_Variables_to_SP, val=cut_lhs)
            context.add(constraint, sense, cut_rhs)
            return

        Transportation_Component_Cost = UB_on_SP - Current_MP_ObjVal
        print(f"SP: {Sequence_Reported_by_SP}, {UB_on_SP} = {Current_MP_ObjVal} + {Transportation_Component_Cost}")




        # If the SP is solved (gap =0) or has gap and its reported LB is worse (larger) than the current best solution
        if Sequence_Reported_by_SP is not None and (LB_on_SP-self.global_UB ) > 0.05:
            print(f"Case 2- SP is not Promising")
            print(f"The node is rejected   ----- ")

            print(f"Feasibility cut on  {Sequence_Reported_by_SP} ")
            cut_lhs, sense, cut_rhs = self.Create_Combinatorial_Cut(Input_Variables_to_SP)
            constraint = cplex.SparsePair(ind=Input_Variables_to_SP, val=cut_lhs)
            context.add(constraint, sense, cut_rhs)
        

            print(f" Optimality cut on  {Sequence_Reported_by_SP} and {LB_on_SP} ")
            cut_lhs, cut_vars, sense, cut_rhs = self.Create_Optimality_Cut(Input_Variables_to_SP, LB_on_SP,Big_M=self.Big_M, LHS_variable="Makespan")
            constraint = cplex.SparsePair(ind=cut_vars, val=cut_lhs)
            context.add(constraint, sense, cut_rhs)



        # If the SP is solved (gap =0) or has gap and its reported UB is better (smaller) than the current best solution
        if Sequence_Reported_by_SP is not None and self.global_UB  >= UB_on_SP :
            print(f"Case 3- SP is Promising ")

            self.global_UB = round(UB_on_SP)
            self.best_solution = Current_MP_Solution
            
            cut_lhs, cut_vars, sense, cut_rhs = self.Create_Optimality_Cut(Input_Variables_to_SP, UB_on_SP,Big_M=self.Big_M, LHS_variable="Makespan")
            constraint = cplex.SparsePair(ind=cut_vars, val=cut_lhs)
            
            # Check whether the constraint is satisfied
            current_values = {name: val for name, val in zip(self.MP_Var_Names, Current_MP_Solution)}
            lhs_value = sum(cut_lhs[i] * current_values.get(cut_vars[i], 0) for i in range(len(cut_vars)))


            if not (sense == 'G' and lhs_value >= cut_rhs - 1e-6):
                print(f" UB updated to {UB_on_SP} and the node is rejected -----")
                print(f" Optimality cut on  {Sequence_Reported_by_SP} and {UB_on_SP} ")

                context.add(constraint, sense, cut_rhs)

            else:
                print(" Optimality cut is satisfied --> not added.")
                print(f"The node is kept   +++++ ")

            
    def Create_Combinatorial_Cut(self, variable_names):
        cut_lhs = [1.0] * len(variable_names)
        sense = 'L'
        cut_rhs = len(variable_names) - 1
        return cut_lhs, sense, cut_rhs


    def Create_Optimality_Cut(self, variable_names, RHS_Value, Big_M, LHS_variable):
        N = len(variable_names)
        cut_vars = [LHS_variable] + variable_names
        cut_lhs = [1.0] + [-Big_M] * N
        sense = 'G'
        cut_rhs = RHS_Value - Big_M * N
        return cut_lhs, cut_vars, sense, cut_rhs



class MyLazyCallback(LazyConstraintCallback):
    def __init__(self, env):
        super().__init__(env)
        self.parent = None 

    def __call__(self):
        MP_solution = self.get_values()
        MP_objval = self.get_objective_value()
        self.parent.lazy_constraint(self, MP_solution, MP_objval,
                                    feasibility_cut=True, optimality_cut=True)
        



def Benders_for_RMS(n, m, q, pt, tt, b_input, b_output, Buffer_Policy, Big_M, Number_of_Thread,
                    LB0_Makespan=0, UB0_Makespan=10000, Time_Limit=3600, SP_Time_Limit=60):
    cpx = Master_Problem(n, m, q, pt, tt, b_input, b_output,
                         LB0_Makespan, UB0_Makespan,Buffer_Policy, Big_M)
    
    cpx.parameters.mip.display.set(3)
    cpx.parameters.threads.set(Number_of_Thread)
    cpx.parameters.mip.strategy.nodeselect.set(1)
    cpx.parameters.mip.strategy.bbinterval.set(1)

    MP_Var_Names = cpx.variables.get_names()
    agvschedulingcuts = MyLazyConstraintCallback(n, m, q, pt, tt, b_input, b_output, Buffer_Policy,
                                                 Big_M, Number_of_Thread,SP_Time_Limit, LB0_Makespan,
                                                 UB0_Makespan, MP_Var_Names)
    


    lazy_cb = cpx.register_callback(MyLazyCallback)
    lazy_cb.parent = agvschedulingcuts


    start = timeit.default_timer()
    cpx.parameters.timelimit.set(Time_Limit)
    cpx.solve()
    stop = timeit.default_timer()
    
    Running_Time = stop - start

    try:

        LB = round(cpx.solution.MIP.get_best_objective())
        UB = round(cpx.solution.get_objective_value())
        Gap = cpx.solution.MIP.get_mip_relative_gap()
        var_values = cpx.solution.get_values()
        Best_Reported_Value_by_Algorithm = agvschedulingcuts.global_UB
        Best_Reported_Solution_by_Algorithm = agvschedulingcuts.best_solution
        
        print("The reported UB is", UB)
        print("The reported LB is", LB)
        print("The gap is:", (UB - LB) / UB)

        Sequence_Reported_by_The_LBBD = [[0 for _ in range(n)] for _ in range(m)]
        for j in range(n):
            for i in range(m):
                for k in range(n):
                    if var_values[j * (m * n) + i * n + k] > 0:
                        Sequence_Reported_by_The_LBBD[i][k] = j + 1

        print("Sequence:", Sequence_Reported_by_The_LBBD)

        return [LB, UB, Gap, Running_Time, var_values, cpx.solution.get_status(),
                cpx.solution.progress.get_num_nodes_processed(),
                cpx.solution.MIP.get_num_cuts(cpx.solution.MIP.cut_type.user),
                agvschedulingcuts.iteration, Best_Reported_Value_by_Algorithm,
                Best_Reported_Solution_by_Algorithm,agvschedulingcuts.Problematic_Case]

    except cplex.exceptions.CplexSolverError as exp:
        print(f"CPLEX Solver Error: {exp}")
        if agvschedulingcuts.global_UB<1000000:
            UB = agvschedulingcuts.global_UB
            LB = cpx.solution.MIP.get_best_objective()
            Best_Reported_Solution_by_Algorithm = agvschedulingcuts.best_solution
            Best_Reported_Value_by_Algorithm = agvschedulingcuts.global_UB
        else:
            UB=1000000
            LB = cpx.solution.MIP.get_best_objective()

            Best_Reported_Solution_by_Algorithm = []
            Best_Reported_Value_by_Algorithm = UB
        Gap = (UB - LB) / UB
        print("The reported UB is", UB)
        print("The reported LB is", LB)
        print("The gap is:", (UB - LB) / UB)
        
        Number_of_Iteration=agvschedulingcuts.iteration

        return [LB, UB, Gap, Running_Time, [], cpx.solution.get_status(),cpx.solution.progress.get_num_nodes_processed(),cpx.solution.MIP.get_num_cuts(cpx.solution.MIP.cut_type.user),Number_of_Iteration,
                 Best_Reported_Value_by_Algorithm,Best_Reported_Solution_by_Algorithm, agvschedulingcuts.Problematic_Case]
