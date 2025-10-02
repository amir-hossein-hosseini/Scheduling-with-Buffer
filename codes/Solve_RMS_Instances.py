# Solving the instances by Single-Tree Serach Logic Based Benders Decomposition (STSLBDD)  
from Class_RMS  import Benders_for_RMS
from Extract_Parameters import *
from Lower_Bounds import *

########################################## Determine if the output is saved as text file or not ##############################################################
Output=1                        #If 1, the solution is oputput in a text file
output_dir = "/.../Solutions"   # Where the output files are saved
Time_Limit=3600
SP_Time_Limit=60          
Number_of_Thread=1
q=1
Big_M=10000
UB0_Makespan=10000
############################################################################################################################################################## 
Buffer_Policy=0         # 0 if the CB exists in the problem (all buffwr ploicies are present)
                        # 1 if the CB is not in the problem (local and MR are present)  
                        # 2 if the MR and CB policies are deactivated (only local buffers are present)
############################################################################################################################################################## 

list1 = [5, 10, 15]
list2 = [2, 3, 4, 5]
list3 = [1, 2, 3, 4, 5,6,7,8,9,10]

for n in list1:
    for m in list2:
        for instance in list3:
            filename = f"/Instance_Directory/{n}-{m} ({instance}).txt"   # replace -Instance_Directory- with the folder containing the instances #
            parameters = Extract_Parameters(filename)
            n=parameters['num_jobs']
            m=parameters['num_machines']
            pt=parameters["processing_times"]
            tt=parameters["transportation_times"]
            b_input=parameters["input_buffer"]
            b_output=parameters["output_buffer"]
            LB0_Makespan=Lower_Bounds(n,m,tt,pt,q)



            [LB, UB, Gap,Running_Time, Var_Array, Status, Num_Nodes, Num_Cuts, Number_of_Iteration,Best_Reported_Value_by_Algorithm,Best_Reported_Solution_by_Algorithm,Problemattic_Case] = Benders_for_RMS(n,m,q, pt,tt, b_input,b_output, Buffer_Policy, Big_M, Number_of_Thread,LB0_Makespan,UB0_Makespan,Time_Limit)
            print("Solution status:                                 %d"% Status)
            print("Nodes processed:                                 %d"% Num_Nodes)
            print("The best found objecetive value (LB)             %d"% LB)
            print("The best feasible solution (UB)                  %d"% UB)
            print("The optimality gap                             ", Gap)
            print('The running time of the algorithm:               %d'% Running_Time)
            print("Active user cuts/lazy constraints:               %d"% Num_Cuts)
            print("Number of performed iterations:                  %d"% Number_of_Iteration)
            print("Number of unresolved subproblems:                %d"% Problemattic_Case)
            print("Variables value:                                  ")
            print(Var_Array)



            if Output==1:
                with open(f"{output_dir}/Solution for {n}-{m}-{q}({instance})-LBBD .txt", "a") as text4:
                    text4.write("\n")
                    text4.write("Instance:\n")
                    text4.write(f"{n}-{m} ({instance})\n")
                    
                    text4.write(f"{LB}\n")
                    text4.write(f"{UB}\n")
                    text4.write(f"{Gap}\n")
                    text4.write(f"{Running_Time}\n") 
                    if  Problemattic_Case>0:     
                        text4.write(f"Unresolved Subproblem\n") 
                    else:
                        text4.write(f"No Unresolved Subproblem\n") 

                    text4.close()
                

                text1= open(f"{output_dir}/A_LB_LBBD.txt", "a")
                text1.write(f"{LB}\n")
                text1.close()

                text2= open(f"{output_dir}/A_UB_LBBD..txt", "a")
                text2.write(f"{UB}\n")
                text2.close()

                text5= open(f"{output_dir}/A_Time_LBBD..txt", "a")
                text5.write(f"{Running_Time}\n")
                text5.close()
                
                text6= open(f"{output_dir}/A_Gap_LBBD.txt", "a")
                text6.write(f"{Gap}\n")
                text6.close()
                
                text6= open(f"{output_dir}/A_Problematic_Cases_LBBD.txt", "a")
                text6.write(f"{Problemattic_Case}\n")
                text6.close()
