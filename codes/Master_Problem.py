# Function: Generating the CPLEX model of the Master problem for the MILP
import cplex

                    
def Master_Problem (n,m, q, pt,tt, b_input,b_output,LB0_Makespan,UB0_Makespan,Buffer_Policy, Big_M):

    Intermedediate_Buffer=[0]*(m-1)
    for i in range (m-1):
        Intermedediate_Buffer[i]=b_input[i+1]+b_output[i]
    print("Intermediate buffers are =",Intermedediate_Buffer)
    
    if Buffer_Policy==0:
        Valid_Inequality=0
    else:
        Valid_Inequality=1

    if Valid_Inequality==1 and all(x == 0 for x in b_input) and all(x == 0 for x in b_output) and q==1:
        permutation=1  
    else:
        permutation=0


    if Buffer_Policy==1:
        b_input = [x + n for x in b_input]

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


    # u_{k,i}: unloading time of job into the kth position of input buffer machine i
        for i in range(1, m + 1):
            for k in range(1, n+ 1):
                varname = f"u_{k},{i}"
                c.variables.add(
                        lb=[0],
                        names=[varname],
                        types=c.variables.type.integer,
                    )
            

    # s_{k,i}: starting time  of job at position k on machine i
        for i in range(1, m + 1):
            for k in range(1, n+ 1):
                varname = f"s_{k},{i}"
                c.variables.add(
                        lb=[0],
                        names=[varname],
                        types=c.variables.type.integer,
                    )

    # b_{k,i}: releasing time  of job at position k into the output buffer of machine i
        for i in range(1, m + 1):
            for k in range(1, n+ 1):
                varname = f"b_{k},{i}"
                c.variables.add(
                        lb=[0],
                        names=[varname],
                        types=c.variables.type.integer,
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


        # Set 3: The starting time of jobs must be greater than  
        # or equal to their completion time on the precedent machine.  
        # s_{k',i+1} >= s_{k,i} + p_{j,i}.x{j,i,k} + tt_{i,i+1} - m.(2-x_{j,k,i}-x_{j,k',i+1})
        for i in range (1,m):
            for j in range(1, n + 1):
                for kk in range(1, n+1):
                    for k in range (1, n+1):
                        var = [] 
                        coe = []
                        var.append(f"s_{kk},{i+1}")
                        coe.append(1.0)
                        var.append(f"s_{k},{i}")
                        coe.append(-1.0)
                        var.append(f"x_{j},{k},{i}")
                        coe.append( - Big_M)
                        var.append(f"x_{j},{kk},{i+1}")
                        coe.append(-Big_M)
                        constraint_name = f"s_{k+1},{i} '>=' b_{k},{i}"
                        c.linear_constraints.add(
                            lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                            senses=["G"],
                            rhs=[+pt[j-1][i-1] +tt[i][i+1] - (2 * Big_M)],
                            names=[constraint_name],
                        )

        # Set 4: the starting time of jobs on machines is larger than their delivery time
        # s_{k,i}>=u{k,i}
        for i in range (1,m+1):
            for k in range (1, n+1):
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

        # Set 5: The starting time of jobs must be greater than or equal to  
        # the release time of their immediate predecessors.  
        # s_{k+1,i} >= b_{k,i}

        for i in range (1,m+1):
            for k in range (1, n):
                var = [] 
                coe = []
                var.append(f"s_{k+1},{i}")
                coe.append(1.0)        
                var.append(f"b_{k},{i}")
                coe.append(-1.0)
                c.linear_constraints.add(
                    lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                    senses=["G"],
                    rhs=[0.0],
                )
        # Set 6: the release time [unloading time of jobs into output buffer of machines] is larger than their starting time + processing time
        # b_{k,i}>= s_{k.i}+ \sum (p_{j,i}.x_{j,k,i})
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

        # Set 7: the release time [unloading time of jobs into output buffer of machines] is larger than the delivery time of precedent job in the output buffer -its transportation time
        # b_{k,i}>= u_{k-b^{out},i+1}-tt_{i,i+1}
        for i in range (1, m):
          for k in range(b_output[i-1]+1, n + 1):
              var = [] 
              coe = []
              var.append(f"b_{k},{i}")
              coe.append(1.0)
              var.append(f"u_{k-b_output[i-1]},{i+1}")
              coe.append(-1.0)
              c.linear_constraints.add(
                  lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                  senses=["G"],
                  rhs=[-tt[i][i+1]],
              )



        # Set 8: the delivery time of jobs into input buffer of machines is larger than their release time  on the previous machine +transportation time 
        # u_{k,i+1}>= b_{k,i}+tt_{i,i+1}
        for i in range (1, m):
            for k in range(1, n + 1):
                var = [] 
                coe = []
                var.append(f"u_{k},{i+1}")
                coe.append(1.0)
                var.append(f"b_{k},{i}")
                coe.append(-1.0)
                c.linear_constraints.add(
                    lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                    senses=["G"],
                    rhs=[tt[i][i+1]],
                )


        # Set 9: the delivery time of jobs into input buffer of machines is larger than the starting time of processing the preceding job in the input buffer 
        # u_{k,i} >= s_{k-b^{in},i}
        for i in range (2, m+1):
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

        # Set 10: the delivery time of jobs into input buffer of machines 1 is 0
        # u_{k,1} = 0
        for k in range(1, n + 1):
            var = [] 
            coe = []
            var.append(f"u_{k},{1}")
            coe.append(1.0)
            c.linear_constraints.add(
                lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                senses=["E"],
                rhs=[0.0],
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
        


        # ### In case that there is only a sibgle MR, the sequence is necessarily permutation 
        if permutation==1 and q==1:
            for j in range(1, n + 1):
                for k in range (1, n + 1):
                    for i in range(1, m):
                        var = [] 
                        coe = []
                        var.append(f"x_{j},{k},{i}")
                        coe.append(1.0)
                        var.append(f"x_{j},{k},{i+1}")
                        coe.append(-1.0)
                        c.linear_constraints.add(
                            lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                            senses=["E"],
                            rhs=[0.0],
                            )
                        
#set x: A job can not be in the q+B earlier/ preceeding positions, because that would contradict buffer capacity
    # i.e., Jobs in positions k (larger than q+B+1) on machine i,   necessarly can not be in postions 1, 2, ..., k-q-B
    # So if x_{j,k,i-1}=1  (for k > q+ B), then sum x{j,1,i}+x_{j,2,i}+ ... + x_{j,k-q-B,i}=0 
    # ===> \sum _{x_j,k',i} <= (1- x_{j,k,i-1})
            
        if Buffer_Policy==1 and Valid_Inequality==1:
            for j in range(1, n + 1):
                for i in range(2, m+1):
                    for k in range (Intermedediate_Buffer[i-2]+q+1, n + 1):
                        var = [] 
                        coe = []
                        for kk in range(1,k-Intermedediate_Buffer[i-2]-q+1):
                            var.append(f"x_{j},{kk},{i}")
                            coe.append(1.0)
                        var.append(f"x_{j},{k},{i-1}")
                        coe.append(+1.0)
                        c.linear_constraints.add(
                            lin_expr=[cplex.SparsePair(ind=var, val=coe)],
                            senses=["L"],
                            rhs=[1],
                            )

    return c
