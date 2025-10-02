#Solving MILP of Robotic Flow Shop without central Buffer and MR 
import cplex
import timeit
from cplex import SparsePair

def SP_Without_CB_Without_MR(n,m,q,tt,pt,b_input,b_output,Big_M,Initial_Variables,LB_on_Obj_Fun,UB_on_Obj_Fun, Number_of_Thread,SP_Time_Limit):

    Mobile_Buffer_Policy=0
    Initial_Solution=1

    Zero_Nodes=[]
    for j in range(1,n+1):  
        outer_list = []
        for i in range(1,m):
            inner_list = [j, i,"0"]
            outer_list.append(inner_list)
        Zero_Nodes.append(outer_list)


    delta_out_for_zero_node=[]
    for j in range(1,n+1):
        for i in range(1,m):
            outer_list = []
            for ii in range(i+1,m):
                inner_list = [j, ii,"0"]
                outer_list.append(inner_list)           
            for jj in range(1,n+1):
                if jj !=j:
                    for ii in range(1,m):
                        inner_list = [jj, ii,"0"]
                        outer_list.append(inner_list)
            inner_list = ["0"]
            outer_list.append(inner_list)
            delta_out_for_zero_node.append(outer_list)

    Cost_of_delta_out_for_zero_node=[]
    for j in range(1,n+1):
        for i in range(1,m):
            outer_list = []
            for node in delta_out_for_zero_node[(j-1)*(m-1)+i-1]:
                if node == ['0']:
                    inner_list=tt[i][i+1]+tt[i+1][0]
                    outer_list.append(inner_list)
                else: 
                    inner_list=tt[i][i+1]+tt[i+1][node[1]]
                    outer_list.append(inner_list)
                                
            Cost_of_delta_out_for_zero_node.append(outer_list)


    delta_in_for_zero_node=[]
    for j in range(1,n+1):
        for i in range(1,m):
            outer_list = []
            inner_list = ["0"]
            outer_list.append(inner_list)
            for ii in range(1,i):
                inner_list = [j, ii,"0"]
                outer_list.append(inner_list)
            for jj in range(1,n+1):
                if jj !=j:
                    for ii in range(1,m):
                        inner_list = [jj, ii,"0"]
                        outer_list.append(inner_list)
            delta_in_for_zero_node.append(outer_list)

    Cost_of_delta_in_for_zero_node=[]
    for j in range(1,n+1):
        for i in range(1,m):
            outer_list = []
            for node in delta_in_for_zero_node[(j-1)*(m-1)+i-1]:
                if node == ['0']:
                    inner_list=tt[0][i]
                    outer_list.append(inner_list)
                else: 
                    inner_list=tt[node[1]][node[1]+1]+tt[node[1]+1][i]
                    outer_list.append(inner_list)
                
                
            Cost_of_delta_in_for_zero_node.append(outer_list)


    delta_out_for_origin=[]

    for j in range(1,n+1):
        for i in range(1,m):
            inner_list = [j, i,"0"]
            delta_out_for_origin.append(inner_list)

    Cost_of_delta_out_for_origin=[]

    for node in delta_out_for_origin:
        inner_list=tt[0][node[1]]
        Cost_of_delta_out_for_origin.append(inner_list)


    delta_in_for_origin=delta_out_for_origin

    Cost_of_delta_in_for_origin=[]
    for node in delta_in_for_origin:
        inner_list=tt[node[1]][node[1]+1]+tt[node[1]+1][0]
        Cost_of_delta_in_for_origin.append(inner_list)
    

    c = cplex.Cplex()

    # Adding variables to model
    # x_{j,k,i}: if job j is assigned to kth position of machine i
    for j in range(1, n + 1):
        for i in range(1, m + 1):
            for k in range(1, n+ 1):
                varname = f"x_{j},{k},{i}"
                c.variables.add(
                    lb=[0],
                    names=[varname],
                    types=c.variables.type.binary,
                )

    # s_{k,i}: starting time of job at position k on machine i
    for i in range(1, m + 1):
        for k in range(1, n+ 1):
            varname = f"s_{k},{i}"
            c.variables.add(
                    lb=[0],
                    names=[varname],
                    types=c.variables.type.integer,
                )

    # b_{k,i}: unloading time  of job at position k into the output buffer of machine i
    for i in range(1, m + 1):
        for k in range(1, n+ 1):
            varname = f"b_{k},{i}"
            c.variables.add(
                    lb=[0],
                    names=[varname],
                    types=c.variables.type.integer,
                )

    # u_{k,i}: unloading time of job at position k into the input buffer of machine i
    for i in range(1, m + 1):
        for k in range(1, n+ 1):
            varname = f"u_{k},{i}"
            c.variables.add(
                    lb=[0],
                    names=[varname],
                    types=c.variables.type.integer,
                )

    # y_{0,v'} for node 0::
    for node in delta_out_for_origin:
        varname = f"y_{['0']},{node}"
        c.variables.add(
        lb=[0], names=[varname], types=c.variables.type.binary
    )

    # y_{v',0} for node 0::
    for node in delta_in_for_origin:
        varname = f"y_{node},{['0']}"
        c.variables.add(
        lb=[0], names=[varname], types=c.variables.type.binary
    )

    # y_{(j,i,0),v'} for zero nodes:
    for j in range(n):
        for i in range(m-1):
            for node in delta_out_for_zero_node[j*(m-1)+i]:
                varname = f"y_{Zero_Nodes[j][i]},{node}"
                c.variables.add(
                lb=[0], names=[varname], types=c.variables.type.binary
                )

    # y_{v',(j,i,0)} for zero nodes:
    for j in range(n):
        for i in range(m-1):
            for node in delta_in_for_zero_node[j*(m-1)+i]:
                varname = f"y_{node},{Zero_Nodes[j][i]}"
                c.variables.add(
                lb=[0], names=[varname], types=c.variables.type.binary
                )


    # f_{j,i,f}: the starting time of the transportation activity 
    for j in range(n):
        for i in range(m-1):
            varname = f"f_{Zero_Nodes[j][i]}"
            c.variables.add(
                lb=[0],
                names=[varname],
                types=c.variables.type.integer,
                )

    varname=f"f_['0']"
    c.variables.add(
        lb=[0],
        names=[varname],
        types=c.variables.type.continuous,
        )


    varname="Makespan"
    c.variables.add(
        lb=[0],
        names=[varname],
        types=c.variables.type.continuous,
        obj=[1],
        )

    # Adding objective function

    c.objective.set_sense(c.objective.sense.minimize)

    # Adding Constraints:
   
    # Set 3: the starting time of jobs must be larger than the starting time of their predecessors 
    for i in range (1,m+1):
        for k in range(1, n):
            var = [None] * (n+2)
            coe = [None] * (n+2)
            var[0] = f"s_{k+1},{i}"
            coe[0] = 1.0
            var[1] = f"s_{k},{i}"
            coe[1] =- 1.0
            h = 2
            for j in range(1, n + 1):
                var[h] = f"x_{j},{k},{i}"
                coe[h] = - pt[j-1][i-1]
                h = h + 1
            c.linear_constraints.add(
                lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                senses=["G"],
                rhs=[0.0],
            )

    # Set 4: the starting time of jobs on machines is larger than their completion time on the previous machine
    for i in range (1,m):
        for j in range(1, n + 1):
            for kk in range(1, n+1):
                for k in range (1, n+1):
                    var = [] 
                    coe = []

                    var.append(f"s_{kk},{i+1}")
                    coe.append(1.0)

                    var.append(f"b_{k},{i}")
                    coe.append(-1.0)

                    var.append(f"x_{j},{k},{i}")
                #  coe.append(-pt[j-1][i-1] - Big_M)
                    coe.append( - Big_M)

                    var.append(f"x_{j},{kk},{i+1}")
                    coe.append(-Big_M)

                    c.linear_constraints.add(
                        lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                        senses=["G"],
                        rhs=[tt[i][i+1] - (2 * Big_M)],
                        #rhs=[ +tt[i][i+1] - (2 * Big_M)],
                    )


    # Set 5-1: at most q number of AGVs are used 

    var = [] 
    coe = []
    for node in delta_out_for_origin:
        var.append(f"y_{['0']},{node}")
        coe.append(1.0)
    c.linear_constraints.add(
        lin_expr=[cplex.SparsePair(ind=var, val=coe)],
        senses=["L"],
        rhs=[q],
    )


    # Set 5-2: If an AGV leaves the depot it should return to depot later.
    var = [] 
    coe = []
    for node in delta_out_for_origin:
        var.append(f"y_{['0']},{node}")
        coe.append(1.0)
    for node in delta_in_for_origin:
        var.append(f"y_{node},{['0']}")
        coe.append(-1.0)
    c.linear_constraints.add(
        lin_expr=[cplex.SparsePair(ind=var, val=coe)],
        senses=["E"],
        rhs=[0.0],
    )


    # Set 6-1: flow conservation constraints 

    for j in range(n):
        for i in range(m-1):
            var = [] 
            coe = []
            for node in delta_out_for_zero_node[j*(m-1)+i]:
                var.append(f"y_{Zero_Nodes[j][i]},{node}")
                coe.append(1.0)
            c.linear_constraints.add(
            lin_expr=[cplex.SparsePair(ind=var, val=coe)],
            senses=["E"],
            rhs=[1.0],
    )

    # Set 6-2: flow conservation constraints 

    for j in range(n):
        for i in range(m-1):
            var = [] 
            coe = []
            for node in delta_out_for_zero_node[j*(m-1)+i]:
                var.append(f"y_{Zero_Nodes[j][i]},{node}")
                coe.append(1.0)
            for node in delta_in_for_zero_node[j*(m-1)+i]:
                var.append(f"y_{node},{Zero_Nodes[j][i]}")
                coe.append(-1.0)
            c.linear_constraints.add(
            lin_expr=[cplex.SparsePair(ind=var, val=coe)],
            senses=["E"],
            rhs=[0.0],
    )


    # Set 7-1: the starting time of the transportation activity node v performed after (j,i,0):

    for j in range(n):
        for i in range(m-1):
            h=0
            for node in delta_out_for_zero_node[j*(m-1)+i]:
                if node != ['0']:
                    var = [] 
                    coe = []
                    var.append(f"f_{node}")
                    coe.append(1.0)
                    var.append(f"f_{Zero_Nodes[j][i]}")
                    coe.append(-1.0)
                    var.append(f"y_{Zero_Nodes[j][i]},{node}")
                    coe.append(-Big_M)
                    c.linear_constraints.add(
                    lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                    senses=["G"],
                    rhs=[-(Big_M)+(Cost_of_delta_out_for_zero_node[j*(m-1)+i][h])],
                    )
                h=h+1

    # Set 7-2: the starting time of the transportation activity node (j,i,0) performed after 0:

    for j in range(n):
        for i in range(m-1):
            var = [] 
            coe = []    
            var.append(f"f_{['0']}")
            coe.append(-1.0)
            var.append(f"f_{Zero_Nodes[j][i]}")
            coe.append(1.0)
            var.append(f"y_{['0']},{Zero_Nodes[j][i]}")
            coe.append(-Big_M)
            c.linear_constraints.add(
            lin_expr=[cplex.SparsePair(ind=var, val=coe)],
            senses=["G"],
            rhs=[-Big_M+tt[0][i+1]],
            )


    # Set 8: the unloading time of jobs into output buffer of machines is larger than their starting time + processing time
    for i in range (1,m+1):
        for k in range(1, n + 1):
            var = [] 
            coe = []
            var.append(f"b_{k},{i}")
            coe.append(1.0)
            var.append(f"s_{k},{i}")
            coe.append(-1.0)
            for j in range(1, n+1):
                var.append(f"x_{j},{k},{i}")
                coe.append(-pt[j-1][i-1])
            c.linear_constraints.add(
                lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                senses=["G"],
                rhs=[0.0],
            )


    # Set 9: the unloading time of jobs into output buffer of machines is larger the starting time of transportaing the preceding job in the output buffer 
    for i in range (1, m):
        for k in range(b_output[i-1]+1, n + 1):
            for j in range(1, n+1):
                var = [] 
                coe = []
                var.append(f"b_{k},{i}")
                coe.append(1.0)
                var.append(f"f_[{j}, {i}, '0']")
                coe.append(-1.0)
                var.append(f"x_{j},{k-b_output[i-1]},{i}")
                coe.append(-Big_M)
                c.linear_constraints.add(
                    lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                    senses=["G"],
                    rhs=[-Big_M],
                )



    # Set 10:  the starting time of processing a job on a machine  is larger than the inloading time of the preceeding job
    for i in range (1, m):
        for k in range(2, n + 1):
            var = [] 
            coe = []
            var.append(f"s_{k},{i}")
            coe.append(1.0)
            var.append(f"b_{k-1},{i}")
            coe.append(-1.0)
            c.linear_constraints.add(
                lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                senses=["G"],
                rhs=[0.0],
            )



    # Set 11:  the starting time of transportation a job on a machine is larger than the starting time of transportation the preceeding job
    for i in range (1, m):
        for j in range (1, n + 1):
            for jj in range (1, n + 1): 
                if jj !=j:
                    for k in range(2, n + 1):
                        for kk in range (1,k):
                            var = [] 
                            coe = []
                            var.append(f"f_[{j}, {i}, '0']")
                            coe.append(1.0)
                            var.append(f"f_[{jj}, {i}, '0']")
                            coe.append(-1.0)
                            var.append(f"x_{j},{k},{i}")
                            coe.append(-Big_M)
                            var.append(f"x_{jj},{kk},{i}")
                            coe.append(-Big_M)
                        c.linear_constraints.add(
                            lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                            senses=["G"],
                            rhs=[-2*Big_M],
                            )


    # Set 12:  the starting time of transportation a job on a machine is larger than its unloading time on the output buffer:
    for i in range (1, m): 
        for k in range(1, n + 1):
                for j in range (1, n + 1):
                    var = [] 
                    coe = []
                    var.append(f"f_[{j}, {i}, '0']")
                    coe.append(1.0)
                    var.append(f"b_{k},{i}")
                    coe.append(-1.0)
                    var.append(f"x_{j},{k},{i}")
                    coe.append(-Big_M)
                    c.linear_constraints.add(
                        lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                        senses=["G"],
                    rhs=[-Big_M],
                        )


    # Set 13: the starting time of processing time is larger than the unloading time of jobs into input  buffer of machines
    for i in range (1,m+1):
        for k in range(1, n + 1):
                    var = [] 
                    coe = []
                    var.append(f"s_{k},{i}")
                    coe.append(1.0)
                    var.append(f"u_{k},{i}")
                    coe.append(-1.0)
                    c.linear_constraints.add(
                        lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                        senses=["G"],
                        rhs=[0.0],
                    )


    # Set 14: the unloading time of jobs into input buffer of machines is larger than the starting time of processing the preceding job in the output buffer 
    for i in range (1, m+1):
        for k in range(b_input[i-1]+1, n + 1):
            var = [] 
            coe = []
            var.append(f"u_{k},{i}")
            coe.append(1.0)
            var.append(f"s_{k-b_input[i-1]},{i}")
            coe.append(-1.0)
            c.linear_constraints.add(
                lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                senses=["G"],
                rhs=[0.0],
            )


    # Set 15: the starting time of transportation is larger than the unloading [into input buffer]  of processing the preceding job + empty travel time
    for j in range(n):
        for i in range(m-1):
            for inner_node in delta_out_for_zero_node[j*(m-1)+i]:
                if inner_node !=['0']:
                    for k in range(1, n + 1):
                        var = [] 
                        coe = []
                        var.append(f"f_{inner_node}")
                        coe.append(1.0)
                        var.append(f"u_{k},{Zero_Nodes[j][i][1]+1}")
                        coe.append(-1.0)
                        var.append(f"y_{Zero_Nodes[j][i]},{inner_node}")
                        coe.append(-Big_M)
                        var.append(f"x_{Zero_Nodes[j][i][0]},{k},{Zero_Nodes[j][i][1]+1}")
                        coe.append(-Big_M)
                        if inner_node[2]=="1":
                            Empty_travel_time=tt[Zero_Nodes[j][i][1]+1][m+1]
                        else: 
                            Empty_travel_time=tt[Zero_Nodes[j][i][1]+1][inner_node[1]]
                        c.linear_constraints.add(
                        lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                        senses=["G"],
                        rhs=[-2*Big_M+Empty_travel_time],
                        )


    # Set 16-1: the unloading [into input buffer] of a job is larger than  the starting time of transportation + empty travel time
    for j in range(n):
        for i in range(m-1):
            for k in range(1, n + 1):
                var = [] 
                coe = []
                var.append(f"u_{k},{Zero_Nodes[j][i][1]+1}")
                coe.append(1.0)
                var.append(f"f_{Zero_Nodes[j][i]}")
                coe.append(-1.0)
                var.append(f"x_{Zero_Nodes[j][i][0]},{k},{Zero_Nodes[j][i][1]+1}")
                coe.append(-Big_M)
        #     for node in delta_in_for_zero_node[j*(m-1)+i]:
        #         var.append(f"y_{node},{Zero_Nodes[j][i]}")
        #         coe.append(-Big_M)  
                Travel_time=tt[Zero_Nodes[j][i][1]][Zero_Nodes[j][i][1]+1]
                c.linear_constraints.add(
                lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                senses=["G"],
            #   rhs=[-2*Big_M+Travel_time],
                rhs=[-Big_M+Travel_time],
                )

    # Set 16-2: activated if AGVs can not be used as the mobile buffer
    if Mobile_Buffer_Policy==0: 
        for j in range(n):
            for i in range(m-1):
                for k in range(1, n + 1):
                    var = [] 
                    coe = []
                    var.append(f"u_{k},{Zero_Nodes[j][i][1]+1}")
                    coe.append(1.0)
                    var.append(f"f_{Zero_Nodes[j][i]}")
                    coe.append(-1.0)
                    var.append(f"x_{Zero_Nodes[j][i][0]},{k},{Zero_Nodes[j][i][1]+1}")
                    coe.append(+Big_M)
                #    for node in delta_in_for_zero_node[j*(m-1)+i]:
                #        var.append(f"y_{node},{Zero_Nodes[j][i]}")
                #        coe.append(+Big_M)  
                    Travel_time=tt[Zero_Nodes[j][i][1]][Zero_Nodes[j][i][1]+1]
                    c.linear_constraints.add(
                    lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                    senses=["L"],
                #   rhs=[+2*Big_M+Travel_time],
                    rhs=[+Big_M+Travel_time],
                    )



    # Obj Fun:: the makespan
    var = [] 
    coe = []
    var.append("Makespan")
    coe.append(1.0)
    var.append(f"b_{n},{m}")
    coe.append(-1.0)
    c.linear_constraints.add(
        lin_expr=[cplex.SparsePair(ind=var, val=coe)],
        senses=["G"],
        rhs=[0.0]
        )

    # Set 22: LB on the makespan
    var = [] 
    coe = []
    var.append(f"Makespan")
    coe.append(1.0)
    c.linear_constraints.add(
        lin_expr=[cplex.SparsePair(ind=var, val=coe)],
        senses=["G"],
        rhs=[LB_on_Obj_Fun],
        )

    # Set 23: UB on the makespan
    var = [] 
    coe = []
    var.append(f"Makespan")
    coe.append(1.0)
    c.linear_constraints.add(
        lin_expr=[cplex.SparsePair(ind=var, val=coe)],
        senses=["L"],
        rhs=[UB_on_Obj_Fun],
        )

    if Initial_Solution==1: 
        # How to add an inital y_{v,v'}----> f"y_{(1,_1,_'0'),(9,_2,_'0')}

   #     Initial_Variables=['x_1,2,1', 'x_1,1,2', 'x_1,1,3', 'x_4,4,1', 'x_4,4,2', 'x_4,3,3', 'x_5,3,1', 'x_5,2,2', 'x_5,7,3', 'x_6,7,1', 'x_6,7,2', 'x_6,6,3', 'x_7,6,1', 'x_7,5,2', 'x_7,4,3']
        Initial_Values=[1] * len(Initial_Variables)
        for var, val in zip(Initial_Variables, Initial_Values):
            c.linear_constraints.add(
                lin_expr=[SparsePair(ind=[var], val=[1.0])],
                senses=["E"],
                rhs=[val],
                names=[f"fix_{var}"]
            )

   # c.write("Model.lp")


    # Solve and output the problem:
    start = timeit.default_timer()
    c.parameters.timelimit.set(SP_Time_Limit)
    c.parameters.mip.display.set(0)
    c.parameters.threads.set(Number_of_Thread)
    

    try:
        c.solve()
        stop = timeit.default_timer()
        SP_Running_Time = stop - start
        


        status = c.solution.get_status()
        if status == c.solution.status.MIP_infeasible:
            return (None,) * 12  # Infeasibility


        UB_on_SP=c.solution.get_objective_value()
        LB_on_SP=c.solution.MIP.get_best_objective()
        Gap_SP=c.solution.MIP.get_mip_relative_gap()



        Job_Positions = [[0 for _ in range(n)] for _ in range(m)]

        Current_SP_Solution=[]
        Current_SP_Value=[]

        Sequence_Reported_by_SP = [[0 for _ in range(n)] for _ in range(m)]


        for j in range(1, n + 1):
            for i in range (1, m+1):
                for k in range(1, n + 1):
                    if c.solution.get_values(f"x_{j},{k},{i}")>0:
                        Job_Positions[i - 1][k - 1]=j
                        Current_SP_Solution.append(f"x_{j},{k},{i}")
                        Current_SP_Value.append(1.0)
                        Sequence_Reported_by_SP[i-1 ][k-1 ]=j


        Position_Based_Delivery_Time = [[0 for _ in range(n)] for _ in range(m)]
        Position_Based_Start_Time = [[0 for _ in range(n)] for _ in range(m)]
        Position_Based_Release_Time = [[0 for _ in range(n)] for _ in range(m)]


        for i in range(1, m + 1): 
            for k in range(1, n + 1):
                u_value = c.solution.get_values(f"u_{k},{i}")
                s_value = c.solution.get_values(f"s_{k},{i}")
                b_value = c.solution.get_values(f"b_{k},{i}")

                Position_Based_Delivery_Time[i - 1][k - 1] = u_value  
                Position_Based_Start_Time[i - 1][k - 1] = s_value  
                Position_Based_Release_Time[i - 1][k - 1] = b_value  

        Job_Based_Leaving_Time=[[None for _ in range(m-1)] for _ in range(n)]
        Position_Based_Leaving_Time=[[None for _ in range(n)] for _ in range(m)]
        Position_Based_Arrival_Time=[[None for _ in range(n)] for _ in range(m)]


        # f_{j,i,f}: the starting time of the transportation activity 
        for j in range(n):
            for i in range(m-1):
                if c.solution.get_values( f"f_{Zero_Nodes[j][i]}")>0.00001:
                    Job_Based_Leaving_Time[j][i]=c.solution.get_values( f"f_{Zero_Nodes[j][i]}")



        for k in range(n):
            for i in range(m-1):
                Position_Based_Leaving_Time[i][k]=Job_Based_Leaving_Time[Job_Positions[i][k]-1][i]
                Position_Based_Arrival_Time[i][k]=Position_Based_Leaving_Time[i][k]+tt[i][i+1]
            Position_Based_Leaving_Time[m-1][k]=Position_Based_Release_Time[m-1][k]
            Position_Based_Arrival_Time[m-1][k]=Position_Based_Release_Time[m-1][k]

        for k in range(n):
            Position_Based_Arrival_Time[0][k]=Position_Based_Delivery_Time[0][k]
            for i in range(1, m):
                Position_Based_Arrival_Time[i][k]=Position_Based_Leaving_Time[i-1][k]+tt[i][i+1]



        Sorted_Processing_Time = [[None for _ in range(n)] for _ in range(m)]

        for k in range(1,n+1):
            for i in range(1, m+1):
                Sorted_Processing_Time[i-1][k - 1]=pt[Job_Positions[i-1][k - 1]-1][i-1]


        return(LB_on_SP,UB_on_SP,Gap_SP,SP_Running_Time,Position_Based_Arrival_Time,Position_Based_Delivery_Time,Position_Based_Start_Time,Position_Based_Release_Time,Position_Based_Leaving_Time,Sequence_Reported_by_SP,Current_SP_Solution,Current_SP_Value)

    except cplex.exceptions.CplexSolverError as e:
        print("SP failed with error:", e)
        return (None,) * 12  # Unexpected error



   
