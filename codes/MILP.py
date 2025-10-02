#Solving MILP of Robotic Flow Shop with Buffer
import cplex
from cplex import SparsePair
import timeit
from Extract_Parameters import *
from Lower_Bounds import *

#############################################  if Output is saved as a text file ####################################
Output=0                        #If 1, the solution is oputput in a text file
output_dir = "/.../Solutions"   # Where the output files are saved
Mobile_Buffer_Policy=1           # If 1, Mrs can be used as buffers
Number_of_Thread=1
Time_Limit=3600
Initial_Solution=0
Big_M=10000
q=1
###################################################################################################################

list1 = [5, 10, 15]
list2 = [2, 3, 4, 5]
list3 = [1, 2, 3, 4, 5,6,7,8,9,10]


for n in list1:
    for m in list2:
        for instance in list3:


            filename = f"Instance_Directory/{n}-{m} ({instance}).txt"  # replace -Instance_Directory- with the folder containing the instances #
            parameters = Extract_Parameters(filename)
            n=parameters['num_jobs']
            m=parameters['num_machines']
            pt=parameters["processing_times"]
            tt=parameters["transportation_times"]
            b_input=parameters["input_buffer"]
            b_output=parameters["output_buffer"]
            

            LB_on_Obj_Fun=Lower_Bounds(n,m,tt,pt,q)
            print("Initial lower bound of the instance=",LB_on_Obj_Fun)


            Zero_Nodes=[]
            for j in range(1,n+1):  
                outer_list = []
                for i in range(1,m):
                    inner_list = [j, i,"0"]
                    outer_list.append(inner_list)
                Zero_Nodes.append(outer_list)

            Minus_One_Nodes=[]
            for j in range(1,n+1):
                outer_list = []
                for i in range(1,m):
                    inner_list = [j, i,"-1"]
                    outer_list.append(inner_list)
                Minus_One_Nodes.append(outer_list)   

            One_Nodes=[]
            for j in range(1,n+1):
                outer_list = []
                for i in range(1,m):
                    inner_list = [j, i,"1"]
                    outer_list.append(inner_list)
                One_Nodes.append(outer_list)   


            delta_out_for_zero_node=[]
            for j in range(1,n+1):
                for i in range(1,m):
                    outer_list = []
                    for ii in range(i+1,m):
                        inner_list = [j, ii,"0"]
                        outer_list.append(inner_list)
                        inner_list = [j, ii,"-1"]
                        outer_list.append(inner_list)
                        inner_list = [j, ii,"1"]
                        outer_list.append(inner_list)
                    for jj in range(1,n+1):
                        if jj !=j:
                            for ii in range(1,m):
                                inner_list = [jj, ii,"0"]
                                outer_list.append(inner_list)
                                inner_list = [jj, ii,"-1"]
                                outer_list.append(inner_list)
                                inner_list = [jj, ii,"1"]
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
                            if node[2]=="1":
                                inner_list=tt[i][i+1]+tt[i+1][m+1]
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
                        inner_list = [j, ii,"-1"]
                        outer_list.append(inner_list)
                        inner_list = [j, ii,"1"]
                        outer_list.append(inner_list)
                    for jj in range(1,n+1):
                        if jj !=j:
                            for ii in range(1,m):
                                inner_list = [jj, ii,"0"]
                                outer_list.append(inner_list)
                                inner_list = [jj, ii,"-1"]
                                outer_list.append(inner_list)
                                inner_list = [jj, ii,"1"]
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
                            if node[2]=="-1":
                                inner_list=tt[node[1]][m+1]+tt[m+1][i]
                                outer_list.append(inner_list)
                            elif node[2]=="1":
                                inner_list=tt[m+1][node[1]+1]+tt[node[1]+1][i]
                                outer_list.append(inner_list)
                            else:
                                inner_list=tt[node[1]][node[1]+1]+tt[node[1]+1][i]
                                outer_list.append(inner_list)
                            
                        
                    Cost_of_delta_in_for_zero_node.append(outer_list)

            delta_out_for_negative_one_node=[]
            for j in range(1,n+1):
                for i in range(1,m):
                    outer_list = []
                    inner_list = [j, i,"1"]
                    outer_list.append(inner_list)
                    for ii in range(i+1,m):
                        inner_list = [j, ii,"0"]
                        outer_list.append(inner_list)
                        inner_list = [j, ii,"-1"]
                        outer_list.append(inner_list)
                        inner_list = [j, ii,"1"]
                        outer_list.append(inner_list)
                    for jj in range(1,n+1):
                        if jj !=j:
                            for ii in range(1,m):
                                inner_list = [jj, ii,"0"]
                                outer_list.append(inner_list)
                                inner_list = [jj, ii,"-1"]
                                outer_list.append(inner_list)
                                inner_list = [jj, ii,"1"]
                                outer_list.append(inner_list)
                    inner_list = ["0"]
                    outer_list.append(inner_list)
                    delta_out_for_negative_one_node.append(outer_list)

            Cost_of_delta_out_for_negative_one_node=[]
            for j in range(1,n+1):
                for i in range(1,m):
                    outer_list = []
                    for node in delta_out_for_negative_one_node[(j-1)*(m-1)+i-1]:
                        if node == ['0']:
                            inner_list=tt[i][m+1]+tt[m+1][0]
                            outer_list.append(inner_list)
                        else: 
                            if node[2]=="1":
                                inner_list=tt[i][m+1]+tt[m+1][m+1]
                                outer_list.append(inner_list)
                            else:
                                inner_list=tt[i][m+1]+tt[m+1][node[1]]
                                outer_list.append(inner_list)
                                        
                    Cost_of_delta_out_for_negative_one_node.append(outer_list)

            delta_in_for_negative_one_node=[]
            for j in range(1,n+1):
                for i in range(1,m):
                    outer_list = []
                    inner_list = ["0"]
                    outer_list.append(inner_list)
                    for ii in range(1,i):
                        inner_list = [j, ii,"0"]
                        outer_list.append(inner_list)
                        inner_list = [j, ii,"-1"]
                        outer_list.append(inner_list)
                        inner_list = [j, ii,"1"]
                        outer_list.append(inner_list)
                    for jj in range(1,n+1):
                        if jj !=j:
                            for ii in range(1,m):
                                inner_list = [jj, ii,"0"]
                                outer_list.append(inner_list)
                                inner_list = [jj, ii,"-1"]
                                outer_list.append(inner_list)
                                inner_list = [jj, ii,"1"]
                                outer_list.append(inner_list)
                    delta_in_for_negative_one_node.append(outer_list)


            Cost_of_delta_in_for_negative_one_node=[]
            for j in range(1,n+1):
                for i in range(1,m):
                    outer_list = []
                    for node in delta_in_for_negative_one_node[(j-1)*(m-1)+i-1]:
                        if node == ['0']:
                            inner_list=tt[0][i]
                            outer_list.append(inner_list)
                        else: 
                            if node[2]=="-1":
                                inner_list=tt[node[1]][m+1]+tt[m+1][i]
                                outer_list.append(inner_list)
                            elif node[2]=="1":
                                inner_list=tt[m+1][node[1]+1]+tt[node[1]+1][i]
                                outer_list.append(inner_list)    
                            else:
                                inner_list=tt[node[1]][node[1]+1]+tt[node[1]+1][i]
                                outer_list.append(inner_list)
                                                    
                    Cost_of_delta_in_for_negative_one_node.append(outer_list)

            delta_out_for_one_node=[]
            for j in range(1,n+1):
                for i in range(1,m):
                    outer_list = []
                    for ii in range(i+1,m):
                        inner_list = [j, ii,"0"]
                        outer_list.append(inner_list)
                        inner_list = [j, ii,"-1"]
                        outer_list.append(inner_list)
                        inner_list = [j, ii,"1"]
                        outer_list.append(inner_list)
                    for jj in range(1,n+1):
                        if jj !=j:
                            for ii in range(1,m):
                                inner_list = [jj, ii,"0"]
                                outer_list.append(inner_list)
                                inner_list = [jj, ii,"-1"]
                                outer_list.append(inner_list)
                                inner_list = [jj, ii,"1"]
                                outer_list.append(inner_list)
                    inner_list = ["0"]
                    outer_list.append(inner_list)
                    delta_out_for_one_node.append(outer_list)


            Cost_of_delta_out_for_one_node=[]
            for j in range(1,n+1):
                for i in range(1,m):
                    outer_list = []
                    for node in delta_out_for_one_node[(j-1)*(m-1)+i-1]:
                        if node == ['0']:
                            inner_list=tt[m+1][i+1]+tt[i+1][0]
                            outer_list.append(inner_list)
                        else: 
                            if node[2]=="1":
                                inner_list=tt[m+1][i+1]+tt[i+1][m+1]
                                outer_list.append(inner_list)
                            else:
                                inner_list=tt[m+1][i+1]+tt[i+1][node[1]]
                                outer_list.append(inner_list)
                    Cost_of_delta_out_for_one_node.append(outer_list)


            delta_in_for_one_node=[]
            for j in range(1,n+1):
                for i in range(1,m):
                    outer_list = []
                    inner_list = ["0"]
                    outer_list.append(inner_list)
                    inner_list = [j, i,"-1"]
                    outer_list.append(inner_list)
                    for ii in range(1,i):
                        inner_list = [j, ii,"0"]
                        outer_list.append(inner_list)
                        inner_list = [j, ii,"-1"]
                        outer_list.append(inner_list)
                        inner_list = [j, ii,"1"]
                        outer_list.append(inner_list)
                    for jj in range(1,n+1):
                        if jj !=j:
                            for ii in range(1,m):
                                inner_list = [jj, ii,"0"]
                                outer_list.append(inner_list)
                                inner_list = [jj, ii,"-1"]
                                outer_list.append(inner_list)
                                inner_list = [jj, ii,"1"]
                                outer_list.append(inner_list)
                    delta_in_for_one_node.append(outer_list)

            Cost_of_delta_in_for_one_node=[]
            for j in range(1,n+1):
                for i in range(1,m):
                    outer_list = []
                    for node in delta_in_for_one_node[(j-1)*(m-1)+i-1]:
                        if node == ['0']:
                            inner_list=tt[0][m+1]
                            outer_list.append(inner_list)
                        else: 
                            if node[2]=="-1":
                                inner_list=tt[node[1]][m+1]+tt[m+1][m+1]
                                outer_list.append(inner_list)
                            elif node[2]=="1":
                                inner_list=tt[m+1][node[1]+1]+tt[node[1]+1][m+1]
                                outer_list.append(inner_list)
                            else:
                                inner_list=tt[node[1]][node[1]+1]+tt[node[1]+1][m+1]
                                outer_list.append(inner_list)

                    Cost_of_delta_in_for_one_node.append(outer_list)


            delta_out_for_origin=[]

            for j in range(1,n+1):
                for i in range(1,m):
                    inner_list = [j, i,"0"]
                    delta_out_for_origin.append(inner_list)
                    inner_list = [j, i,"-1"]
                    delta_out_for_origin.append(inner_list)
                    inner_list = [j, i,"1"]
                    delta_out_for_origin.append(inner_list)   

            Cost_of_delta_out_for_origin=[]

            for node in delta_out_for_origin:
                if node[2]=="1":
                    inner_list=tt[0][m+1]
                    Cost_of_delta_out_for_origin.append(inner_list)
                else:
                    inner_list=tt[0][node[1]]
                    Cost_of_delta_out_for_origin.append(inner_list)


            delta_in_for_origin=delta_out_for_origin

            Cost_of_delta_in_for_origin=[]
            for node in delta_in_for_origin:
                if node[2]=="1":
                    inner_list=tt[m+1][node[1]+1]+tt[node[1]+1][0]
                    Cost_of_delta_in_for_origin.append(inner_list)
                elif node[2]=="-1":
                    inner_list=tt[node[1]][m+1]+tt[m+1][0]
                    Cost_of_delta_in_for_origin.append(inner_list)
                else: 
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


            # y_{(j,i,-1),v'} for negative nodes:
            for j in range(n):
                for i in range(m-1):
                    for node in delta_out_for_negative_one_node[j*(m-1)+i]:
                        varname = f"y_{Minus_One_Nodes[j][i]},{node}"
                        c.variables.add(
                        lb=[0], names=[varname], types=c.variables.type.binary
                        )

            # y_{v',(j,i,-1)} for negative nodes:
            for j in range(n):
                for i in range(m-1):
                    for node in delta_in_for_negative_one_node[j*(m-1)+i]:
                        varname = f"y_{node},{Minus_One_Nodes[j][i]}"
                        c.variables.add(
                        lb=[0], names=[varname], types=c.variables.type.binary
                        )

            # y_{(j,i,+1),v'} for one nodes:
            for j in range(n):
                for i in range(m-1):
                    for node in delta_out_for_one_node[j*(m-1)+i]:
                        varname = f"y_{One_Nodes[j][i]},{node}"
                        c.variables.add(
                        lb=[0], names=[varname], types=c.variables.type.binary
                        )

            # y_{v',(j,i,+1)} for one nodes:
            for j in range(n):
                for i in range(m-1):
                    for node in delta_in_for_one_node[j*(m-1)+i]:
                        varname = f"y_{node},{One_Nodes[j][i]}"
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


            for j in range(n):
                for i in range(m-1):
                    varname = f"f_{One_Nodes[j][i]}"
                    c.variables.add(
                        lb=[0],
                        names=[varname],
                        types=c.variables.type.integer,
                        )     


            for j in range(n):
                for i in range(m-1):
                    varname = f"f_{Minus_One_Nodes[j][i]}"
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

            varname="Flow_Time"
            c.variables.add(
                lb=[0],
                names=[varname],
                types=c.variables.type.continuous,
                obj=[1],
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
            # Set 1: every job must be assigned to exactly position on each machine:
            for j in range(1, n + 1):
                for i in range (1, m+1):
                    var = [None] * (n)
                    coe = [None] * (n)
                    h = 0
                    for k in range(1, n + 1):
                        var[h] = f"x_{j},{k},{i}"
                        coe[h] = 1.0
                        h = h + 1
                    c.linear_constraints.add(
                        lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                        senses=["E"],
                        rhs=[1.0],
                    )

            # Set 2: every position must be assigned to exactly job on each machine:
            for i in range (1,m+1):
                for k in range(1, n + 1):
                    var = [None] * (n)
                    coe = [None] * (n)
                    h = 0
                    for j in range(1, n + 1):
                        var[h] = f"x_{j},{k},{i}"
                        coe[h] = 1.0
                        h = h + 1
                    c.linear_constraints.add(
                        lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                        senses=["E"],
                        rhs=[1.0],
                    )


            # Set 3: the starting time of jobs on the first machines must be larger than the release times 
            #for k in range(1, n + 1):
            #    var = [None] * (n+1)
            #    coe = [None] * (n+1)
            #    var[0] = f"s_{k},{1}"
            #    coe[0] = 1.0
            #    h = 1
            #    for j in range(1, n + 1):
            #        var[h] = f"x_{j},{k},{1}"
            #        coe[h] = - release_time[j-1]
            #        h = h + 1
            #    c.linear_constraints.add(
            #        lin_expr=[cplex.SparsePair(ind=var, val=coe)],
            #        senses=["G"],
            #        rhs=[0.0],
            #    )

            # Set 4: the starting time of jobs must be larger than the starting time of their predecessors 
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

            # Set 5: the starting time of jobs on machines is larger than their completion time on the previous machine
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


            # Set 6: at most q number of AGVs are used 

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


            # Set 7: If an AGV leaves the depot it should return to depot later.
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


            # Set 8-1: flow conservation constraints 

            for j in range(n):
                for i in range(m-1):
                    var = [] 
                    coe = []
                    for node in delta_out_for_zero_node[j*(m-1)+i]:
                        var.append(f"y_{Zero_Nodes[j][i]},{node}")
                        coe.append(1.0)
                    c.linear_constraints.add(
                    lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                    senses=["L"],
                    rhs=[1.0],
            )

            # Set 8-2: flow conservation constraints 

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

            # Set 8-3: flow conservation constraints 

            for j in range(n):
                for i in range(m-1):
                    var = [] 
                    coe = []
                    for node in delta_out_for_negative_one_node[j*(m-1)+i]:
                        var.append(f"y_{Minus_One_Nodes[j][i]},{node}")
                        coe.append(1.0)
                    c.linear_constraints.add(
                    lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                    senses=["L"],
                    rhs=[1.0],
            )

            # Set 8-4: flow conservation constraints 

            for j in range(n):
                for i in range(m-1):
                    var = [] 
                    coe = []
                    for node in delta_out_for_negative_one_node[j*(m-1)+i]:
                        var.append(f"y_{Minus_One_Nodes[j][i]},{node}")
                        coe.append(1.0)
                    for node in delta_in_for_negative_one_node[j*(m-1)+i]:
                        var.append(f"y_{node},{Minus_One_Nodes[j][i]}")
                        coe.append(-1.0)

                    c.linear_constraints.add(
                    lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                    senses=["E"],
                    rhs=[0.0],
            )

            # Set 8-5: flow conservation constraints 

            for j in range(n):
                for i in range(m-1):
                    var = [] 
                    coe = []
                    for node in delta_out_for_one_node[j*(m-1)+i]:
                        var.append(f"y_{One_Nodes[j][i]},{node}")
                        coe.append(1.0)
                    c.linear_constraints.add(
                    lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                    senses=["L"],
                    rhs=[1.0],
            )

            # Set 8-6: flow conservation constraints 

            for j in range(n):
                for i in range(m-1):
                    var = [] 
                    coe = []
                    for node in delta_out_for_one_node[j*(m-1)+i]:
                        var.append(f"y_{One_Nodes[j][i]},{node}")
                        coe.append(1.0)
                    for node in delta_in_for_one_node[j*(m-1)+i]:
                        var.append(f"y_{node},{One_Nodes[j][i]}")
                        coe.append(-1.0)
                    c.linear_constraints.add(
                    lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                    senses=["E"],
                    rhs=[0.0],
            )


            # Set 9: Only one of (j,i,0) or (j,i,-1) is selected

            for j in range(n):
                for i in range(m-1):
                    var = [] 
                    coe = []
                    for node in delta_in_for_zero_node[j*(m-1)+i]:
                        var.append(f"y_{node},{Zero_Nodes[j][i]}")
                        coe.append(1.0)
                    for node in delta_in_for_negative_one_node[j*(m-1)+i]:
                        var.append(f"y_{node},{Minus_One_Nodes[j][i]}")
                        coe.append(1.0)
                    c.linear_constraints.add(
                    lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                    senses=["E"],
                    rhs=[1.0],
            )

            # Set 10: Only one of (j,i,0) or (j,i,+1) is selected

            for j in range(n):
                for i in range(m-1):
                    var = [] 
                    coe = []
                    for node in delta_in_for_zero_node[j*(m-1)+i]:
                        var.append(f"y_{node},{Zero_Nodes[j][i]}")
                        coe.append(1.0)
                    for node in delta_in_for_one_node[j*(m-1)+i]:
                        var.append(f"y_{node},{One_Nodes[j][i]}")
                        coe.append(1.0)
                    c.linear_constraints.add(
                    lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                    senses=["E"],
                    rhs=[1.0],
            )

            # Set 11-1: the starting time of the transportation activity node v performed after (j,i,0):

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

            # Set 11-2: the starting time of the transportation activity node (j,i,0) performed after 0:

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



            # Set 11-2: the starting time of the transportation activity node (j,i,0) performed after v:
            #for j in range(n):
            #    for i in range(m-1):
            #        h=0
            #        for node in delta_in_for_zero_node[j*(m-1)+i]:
            #            var = [] 
            #            coe = []
            #            var.append(f"f_{node}")
            #            coe.append(-1.0)
            #            var.append(f"f_{Zero_Nodes[j][i]}")
            #            coe.append(1.0)
            #            var.append(f"y_{node},{Zero_Nodes[j][i]}")
            #            coe.append(-Big_M)
            #            c.linear_constraints.add(
            #            lin_expr=[cplex.SparsePair(ind=var, val=coe)],
            #            senses=["G"],
            #            rhs=[-(Big_M)+(Cost_of_delta_in_for_zero_node[j*(m-1)+i][h])],
            #            )
            #            h=h+1


            # Set 11-3: the starting time of the transportation activities (j,i,1):
            for j in range(n):
                for i in range(m-1):
                    h=0
                    for node in delta_out_for_one_node[j*(m-1)+i]:
                        if node != ['0']:
                            var = [] 
                            coe = []
                            var.append(f"f_{node}")
                            coe.append(1.0)
                            var.append(f"f_{One_Nodes[j][i]}")
                            coe.append(-1.0)
                            var.append(f"y_{One_Nodes[j][i]},{node}")
                            coe.append(-Big_M)
                            c.linear_constraints.add(
                            lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                            senses=["G"],
                            rhs=[-(Big_M)+(Cost_of_delta_out_for_one_node[j*(m-1)+i][h])],
                            )
                        h=h+1

            # Set 11-4: the starting time of the transportation activity node (j,i,+1) performed after 0:
            for j in range(n):
                for i in range(m-1):
                    var = [] 
                    coe = []    
                    var.append(f"f_{['0']}")
                    coe.append(-1.0)
                    var.append(f"f_{One_Nodes[j][i]}")
                    coe.append(1.0)
                    var.append(f"y_{['0']},{One_Nodes[j][i]}")
                    coe.append(-Big_M)
                    c.linear_constraints.add(
                    lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                    senses=["G"],
                    rhs=[-Big_M+tt[0][m+1]],
                    )




            # Set 11-4: the starting time of the transportation activity node (j,i,+1) performed after v:
            #for j in range(n):
            #    for i in range(m-1):
            #        h=0
            #        for node in delta_in_for_one_node[j*(m-1)+i]:
            #            var = [] 
            #            coe = []
            #            var.append(f"f_{node}")
            #            coe.append(-1.0)
            #            var.append(f"f_{One_Nodes[j][i]}")
            #            coe.append(1.0)
            #            var.append(f"y_{node},{One_Nodes[j][i]}")
            #            coe.append(-Big_M)
            #            c.linear_constraints.add(
            #            lin_expr=[cplex.SparsePair(ind=var, val=coe)],
            #            senses=["G"],
            #            rhs=[-(Big_M)+(Cost_of_delta_in_for_one_node[j*(m-1)+i][h])],
            #            )
            #            h=h+1



            # Set 11-5: the starting time of the transportation activities (j,i,-1):

            for j in range(n):
                for i in range(m-1):
                    h=0
                    for node in delta_out_for_negative_one_node[j*(m-1)+i]:
                        if node != ['0']:
                            var = [] 
                            coe = []
                            var.append(f"f_{node}")
                            coe.append(1.0)
                            var.append(f"f_{Minus_One_Nodes[j][i]}")
                            coe.append(-1.0)
                            var.append(f"y_{Minus_One_Nodes[j][i]},{node}")
                            coe.append(-Big_M)
                            c.linear_constraints.add(
                            lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                            senses=["G"],
                            rhs=[-(Big_M)+(Cost_of_delta_out_for_negative_one_node[j*(m-1)+i][h])],
                            )
                        h=h+1


            # Set 11-6: the starting time of the transportation activity node (j,i,-1) performed after 0:
            for j in range(n):
                for i in range(m-1):
                    var = [] 
                    coe = []    
                    var.append(f"f_{['0']}")
                    coe.append(-1.0)
                    var.append(f"f_{Minus_One_Nodes[j][i]}")
                    coe.append(1.0)
                    var.append(f"y_{['0']},{Minus_One_Nodes[j][i]}")
                    coe.append(-Big_M)
                    c.linear_constraints.add(
                    lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                    senses=["G"],
                    rhs=[-Big_M+tt[0][i+1]],
                    )


            # Set 11-6: the starting time of the transportation activity node (j,i,-1) performed after v:
            #for j in range(n):
            #    for i in range(m-1):
            #        h=0
            #        for node in delta_in_for_negative_one_node[j*(m-1)+i]:
            #            var = [] 
            #            coe = []
            #            var.append(f"f_{node}")
            #            coe.append(-1.0)
            #            var.append(f"f_{Minus_One_Nodes[j][i]}")
            #            coe.append(1.0)
            #            var.append(f"y_{node},{Minus_One_Nodes[j][i]}")
            #            coe.append(-Big_M)
            #            c.linear_constraints.add(
            #            lin_expr=[cplex.SparsePair(ind=var, val=coe)],
            #            senses=["G"],
            #            rhs=[-(Big_M)+(Cost_of_delta_in_for_negative_one_node[j*(m-1)+i][h])],
            #            )
            #            h=h+1




            # Set 12: the starting time of the transportation activities (j,i,1) is larger than the starting time of (j,i,-1):
            for j in range(n):
                for i in range(m-1):
                    var = [] 
                    coe = []
                    var.append(f"f_{One_Nodes[j][i]}")
                    coe.append(1.0)
                    var.append(f"f_{Minus_One_Nodes[j][i]}")
                    coe.append(-1.0)
                    c.linear_constraints.add(
                        lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                        senses=["G"],
                        rhs=[tt[Minus_One_Nodes[j][i][1]][m+1]],
                        )

            # Set 15: the unloading time of jobs into output buffer of machines is larger than their starting time + processing time
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


            # Set 16: the unloading time of jobs into output buffer of machines is larger the starting time of transportaing the preceding job in the output buffer 
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


            # Set 17: the unloading time of jobs into output buffer of machines is larger the starting time of transportaing the preceding job in the output buffer 
            for i in range (1, m):
                for k in range(b_output[i-1]+1, n + 1):
                            for j in range(1, n+1):
                                var = [] 
                                coe = []
                                var.append(f"b_{k},{i}")
                                coe.append(1.0)
                                var.append(f"f_[{j}, {i}, '-1']")
                                coe.append(-1.0)
                                var.append(f"x_{j},{k-b_output[i-1]},{i}")
                                coe.append(-Big_M)
                                c.linear_constraints.add(
                                    lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                                    senses=["G"],
                                    rhs=[-Big_M],
                                )


            # Set 18:  the starting time of processing a job on a machine  is larger than the inloading time of the preceeding job
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


            # Set 19-1:  the starting time of transportation a job on a machine is larger than the starting time of transportation the preceeding job
            #for i in range (1, m):
            #    for k in range(2, n + 1):
            #        for kk in range (1,k):
            #            for j in range (1, n + 1):
            #                for jj in range (1, n + 1): 
            #                    if jj !=j:
            #                        var = [] 
            #                        coe = []
            #                        var.append(f"f_[{j}, {i}, '0']")
            #                        coe.append(1.0)
            #                        var.append(f"f_[{jj}, {i}, '0']")
            #                        coe.append(-1.0)
            #                        var.append(f"x_{j},{k},{i}")
            #                        coe.append(-Big_M)
            #                        var.append(f"x_{jj},{kk},{i}")
            #                        coe.append(-Big_M)
            #                        c.linear_constraints.add(
            #                            lin_expr=[cplex.SparsePair(ind=var, val=coe)],
            #                            senses=["G"],
            #                        rhs=[-2*Big_M],
            #                            )
                                    

            # Set 19-1:  the starting time of transportation a job on a machine is larger than the starting time of transportation the preceeding job
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

            # Set 19-2:  the starting time of transportation a job on a machine is larger than the starting time of transportation the preceeding job
            #for i in range (1, m):
            #    for k in range(2, n + 1):
            #        for kk in range (1,k):
            #            for j in range (1, n + 1):
            #                for jj in range (1, n + 1): 
            #                    if jj !=j:
            #                        var = [] 
            #                        coe = []
            #                        var.append(f"f_[{j}, {i}, '0']")
            #                        coe.append(1.0)
            #                        var.append(f"f_[{jj}, {i}, '-1']")
            #                        coe.append(-1.0)
            #                        var.append(f"x_{j},{k},{i}")
            #                        coe.append(-Big_M)
            #                        var.append(f"x_{jj},{kk},{i}")
            #                        coe.append(-Big_M)
            #                        c.linear_constraints.add(
            #                        lin_expr=[cplex.SparsePair(ind=var, val=coe)],
            #                        senses=["G"],
            #                        rhs=[-2*Big_M],
            #                        )

            # Set 19-2:  the starting time of transportation a job on a machine is larger than the starting time of transportation the preceeding job
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
                                    var.append(f"f_[{jj}, {i}, '-1']")
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
                                
            # Set 19-3:  the starting time of transportation a job on a machine is larger than the starting time of transportation the preceeding job
            #for i in range (1, m):
            #    for k in range(2, n + 1):
            #        for kk in range (1,k):
            #            for j in range (1, n + 1):
            #                for jj in range (1, n + 1): 
            #                    if jj !=j:
            #                        var = [] 
            #                        coe = []
            #                        var.append(f"f_[{j}, {i}, '-1']")
            #                        coe.append(1.0)
            #                        var.append(f"f_[{jj}, {i}, '0']")
            #                        coe.append(-1.0)
            #                        var.append(f"x_{j},{k},{i}")
            #                        coe.append(-Big_M)
            #                        var.append(f"x_{jj},{kk},{i}")
            #                        coe.append(-Big_M)
            #                        c.linear_constraints.add(
            #                            lin_expr=[cplex.SparsePair(ind=var, val=coe)],
            #                            senses=["G"],
            #                        rhs=[-2*Big_M],
            #                            )


            # Set 19-3:  the starting time of transportation a job on a machine is larger than the starting time of transportation the preceeding job
            for i in range (1, m):
                for j in range (1, n + 1):
                    for jj in range (1, n + 1): 
                        if jj !=j:
                            for k in range(2, n + 1):
                                for kk in range (1,k):
                                    var = [] 
                                    coe = []
                                    var.append(f"f_[{j}, {i}, '-1']")
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

            # Set 19-4:  the starting time of transportation a job on a machine is larger than the starting time of transportation the preceeding job
            #for i in range (1, m):
            #    for k in range(2, n + 1):
            #        for kk in range (1,k):
            #            for j in range (1, n + 1):
            #                for jj in range (1, n + 1): 
            #                    if jj !=j:
            #                        var = [] 
            #                        coe = []
            #                        var.append(f"f_[{j}, {i}, '-1']")
            #                        coe.append(1.0)
            #                        var.append(f"f_[{jj}, {i}, '-1']")
            #                        coe.append(-1.0)
            #                        var.append(f"x_{j},{k},{i}")
            #                        coe.append(-Big_M)
            #                        var.append(f"x_{jj},{kk},{i}")
            #                        coe.append(-Big_M)
            #                        c.linear_constraints.add(
            #                            lin_expr=[cplex.SparsePair(ind=var, val=coe)],
            #                            senses=["G"],
            #                        rhs=[-2*Big_M],
            #                           )


            # Set 19-4:  the starting time of transportation a job on a machine is larger than the starting time of transportation the preceeding job
            for i in range (1, m):
                for j in range (1, n + 1):
                    for jj in range (1, n + 1): 
                        if jj !=j:
                            for k in range(2, n + 1):
                                for kk in range (1,k):
                                    var = [] 
                                    coe = []
                                    var.append(f"f_[{j}, {i}, '-1']")
                                    coe.append(1.0)
                                    var.append(f"f_[{jj}, {i}, '-1']")
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


            # Set 20-1:  the starting time of transportation a job on a machine is larger than its unloading time on the output buffer:
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

            # Set 20-2:  the starting time of transportation a job on a machine is larger than its unloading time on the output buffer:
            for i in range (1, m): 
                for k in range(1, n + 1):
                        for j in range (1, n + 1):
                            var = [] 
                            coe = []
                            var.append(f"f_[{j}, {i}, '-1']")
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

            # Set 21: the starting time of processing time is larger than the unloading time of jobs into input  buffer of machines
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

            # Set 22: the unloading time of jobs into input buffer of machines is larger than the starting time of processing the preceding job in the output buffer 
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

            # Set 23-1: the starting time of transportation is larger than the unloading [into input buffer]  of processing the preceding job + empty travel time
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

            # Set 23-2: the starting time of transportation is larger than the unloading [into unput buffer]  of the preceding job + empty travel time
            for j in range(n):
                for i in range(m-1):
                    for inner_node in delta_out_for_one_node[j*(m-1)+i]:
                        if inner_node !=['0']:
                            for k in range(1, n + 1):
                                var = [] 
                                coe = []
                                var.append(f"f_{inner_node}")
                                coe.append(1.0)
                                var.append(f"u_{k},{One_Nodes[j][i][1]+1}")
                                coe.append(-1.0)
                                var.append(f"y_{One_Nodes[j][i]},{inner_node}")
                                coe.append(-Big_M)
                                var.append(f"x_{One_Nodes[j][i][0]},{k},{One_Nodes[j][i][1]+1}")
                                coe.append(-Big_M)
                                if inner_node[2]=="1":
                                    Empty_travel_time=tt[One_Nodes[j][i][1]+1][m+1]
                                else: 
                                    Empty_travel_time=tt[One_Nodes[j][i][1]+1][inner_node[1]]
                                c.linear_constraints.add(
                                lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                                senses=["G"],
                                rhs=[-2*Big_M+Empty_travel_time],
                                )


            # Set 24-1: the unloading [into input buffer] of a job is larger than  the starting time of transportation + empty travel time
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
                        for node in delta_in_for_zero_node[j*(m-1)+i]:
                            var.append(f"y_{node},{Zero_Nodes[j][i]}")
                            coe.append(-Big_M)  
                        Travel_time=tt[Zero_Nodes[j][i][1]][Zero_Nodes[j][i][1]+1]
                        c.linear_constraints.add(
                        lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                        senses=["G"],
                        rhs=[-2*Big_M+Travel_time],
                        )

            # Set 24-2: activated if AGVs can not be used as the mobile buffer
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
                            for node in delta_in_for_zero_node[j*(m-1)+i]:
                                var.append(f"y_{node},{Zero_Nodes[j][i]}")
                                coe.append(+Big_M)  
                            Travel_time=tt[Zero_Nodes[j][i][1]][Zero_Nodes[j][i][1]+1]
                            c.linear_constraints.add(
                            lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                            senses=["L"],
                            rhs=[+2*Big_M+Travel_time],
                            )

            # Set 25-1: the unloading [into input buffer] of a job is larger than  the starting time of transportation + empty travel time
            for j in range(n):
                for i in range(m-1):
                    for k in range(1, n + 1):
                        var = [] 
                        coe = []
                        var.append(f"u_{k},{One_Nodes[j][i][1]+1}")
                        coe.append(1.0)
                        var.append(f"f_{One_Nodes[j][i]}")
                        coe.append(-1.0)
                        var.append(f"x_{One_Nodes[j][i][0]},{k},{One_Nodes[j][i][1]+1}")
                        coe.append(-Big_M)  
                        for node in delta_in_for_one_node[j*(m-1)+i]:
                            var.append(f"y_{node},{One_Nodes[j][i]}")
                            coe.append(-Big_M)
                        Travel_time=tt[m+1][One_Nodes[j][i][1]+1]
                        c.linear_constraints.add(
                        lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                        senses=["G"],
                        rhs=[-2*Big_M+Travel_time],
                        )


            # Set 25-2: activated if AGVs can not be used as the mobile buffer
            if Mobile_Buffer_Policy==0: 
                for j in range(n):
                    for i in range(m-1):
                        for k in range(1, n + 1):
                            var = [] 
                            coe = []
                            var.append(f"u_{k},{One_Nodes[j][i][1]+1}")
                            coe.append(1.0)
                            var.append(f"f_{One_Nodes[j][i]}")
                            coe.append(-1.0)
                            var.append(f"x_{One_Nodes[j][i][0]},{k},{One_Nodes[j][i][1]+1}")
                            coe.append(+Big_M)  
                            for node in delta_in_for_one_node[j*(m-1)+i]:
                                var.append(f"y_{node},{One_Nodes[j][i]}")
                                coe.append(+Big_M)
                            Travel_time=tt[m+1][One_Nodes[j][i][1]+1]
                            c.linear_constraints.add(
                            lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                            senses=["L"],
                            rhs=[+Big_M+Travel_time],
                            )


            # Obj Fun:: the completion time
            #for j in  range(1, n + 1):
            #    for k in  range(1, n + 1):
            #        var = [] 
            #        coe = []
            #        var.append(f"c_{j}")
            #        coe.append(1.0)
            #        var.append(f"b_{k},{m}")
            #        coe.append(-1.0)
            #        var.append(f"x_{j},{k},{m}")
            #        coe.append(-Big_M)  
            #        c.linear_constraints.add(
            #            lin_expr=[cplex.SparsePair(ind=var, val=coe)],
            #            senses=["G"],
                    #  rhs=[-Big_M+pt[j-1][m-1]],
            #            rhs=[-Big_M]
            #            )


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



            # Give initial values to the variables

            #for i in range (1, m+1):
            #    var = [] 
            #    coe = []
            #    var.append(f"x_{1},{1},{i}")
            #    coe.append(1.0)
            #    c.linear_constraints.add(
            #        lin_expr=[cplex.SparsePair(ind=var, val=coe)],
            #        senses=["E"],
            #        rhs=[1.0],
            #        )


            if Initial_Solution==1:
                # How the initial solution looks like:
                Initial_Variables= ['x_1,2,1', 'x_1,4,2', 'x_1,2,3', 'x_1,2,4','x_2,4,1', 'x_2,3,2', 'x_2,5,3',  'x_2,3,4', 'x_3,5,1', 'x_3,2,2', 'x_3,3,3',  'x_3,1,4','x_4,1,1', 'x_4,5,2', 'x_4,1,3', 'x_4,4,4', 'x_5,3,1', 'x_5,1,2', 'x_5,4,3','x_5,5,4']
                Initial_Values= [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

                for var, val in zip(Initial_Variables, Initial_Values):
                    c.linear_constraints.add(
                        lin_expr=[SparsePair(ind=[var], val=[1.0])],
                        senses=["E"],
                        rhs=[val],
                        names=[f"fix_{var}"]
                    )



            # Solve and output the problem:
            start = timeit.default_timer()
            c.parameters.timelimit.set(Time_Limit)
            c.parameters.mip.display.set(2)
            c.parameters.threads.set(Number_of_Thread)

            c.solve()
            stop = timeit.default_timer()
            time = stop - start

            try:

                UB=c.solution.get_objective_value()
                LB=c.solution.MIP.get_best_objective()
                Gap=c.solution.MIP.get_mip_relative_gap()
                time = stop - start

                Job_Positions = [[0 for _ in range(n)] for _ in range(m)]

                print("LB for the problem=",LB)
                print("UB for the problem=",UB)
                print("Gap =", Gap)
                print("Total elapsed time=", time)      




            except cplex.exceptions.CplexSolverError as exp:
                print(f"CPLEX Solver Error: {exp}")
                LB = c.solution.MIP.get_best_objective()
                UB=1000000
                Best_Reported_Solution_by_Algorithm = []
                Best_Reported_Value_by_Algorithm = UB
                Gap = (UB - LB) / UB
                print("LB for the problem=",LB)
                print("UB for the problem=",UB)
                print("Gap =", Gap)
                print("Total elapsed time=", time)      




            if Output==1:
                output_file = f"{output_dir}/Solution for {n}-{m} ({instance}).txt"

                with open(f"{output_dir}/Solution for {n}-{m} ({instance})-MILP.txt", "a") as text4:
                    text4.write("\n")
                    text4.write("Instance:\n")
                    text4.write(f"{n}-{m} ({instance}\n")
                    
                    text4.write(f"{LB}\n")
                    text4.write(f"{UB}\n")
                    text4.write(f"{Gap}\n")
                    text4.write(f"{time}\n")           
                    text4.close()
                

                text1= open(f"{output_dir}/A_LB_MILP.txt", "a")
                text1.write(f"{LB}\n")
                text1.close()

                text2= open(f"{output_dir}/A_UB_MILP.txt", "a")
                text2.write(f"{UB}\n")
                text2.close()

                text5= open(f"{output_dir}/A_Time_MILP.txt", "a")
                text5.write(f"{time}\n")
                text5.close()
                
                text6= open(f"{output_dir}/A_Gap_MILP.txt", "a")
                text6.write(f"{Gap}\n")
                text6.close()
