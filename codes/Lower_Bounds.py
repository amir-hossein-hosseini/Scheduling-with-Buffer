# Function: Computes the initial lower bound of the RMS problem
import math

def Lower_Bounds (n,m,tt,pt,q):


    Total_pt_of_Jobs=[]
    for j in range(n):
        Sum=0
        for i in range (m):
            Sum=pt[j][i]+Sum
        Total_pt_of_Jobs.append(Sum)

    Max_Total_pt_Time_of_Jobs=max(Total_pt_of_Jobs)


    Total_tr_Time=0
    for i in range (m-1):
        Total_tr_Time=Total_tr_Time+tt[i+1][i+2]


    LB1=Max_Total_pt_Time_of_Jobs+Total_tr_Time

 #   print("LB1=",LB1)


    Total_pt_of_Machines=[]
    for i in range(m):
        Sum=0
        for j in range (n):
            Sum=pt[j][i]+Sum
        Total_pt_of_Machines.append(Sum)
 #   print("Total processing time on machines=",Total_pt_of_Machines)

    Max_Total_pt_Time_of_Machines=max(Total_pt_of_Machines)

    LB2_1=Max_Total_pt_Time_of_Machines+Total_tr_Time

  #  print("LB2_1=",LB2_1)

    Head=[0]*m
    Tail=[0]*m

    for i in range (1,m):  # The first machine has no head
        for k in range(i):
            column = [row[k] for row in pt]
            Head[i]=min(column)+Head[i]

  #  print("Head of machines=",Head)


    for i in range (m-1):  # The first machine has no head
        for k in range(i+1,m):
            column = [row[k] for row in pt]
            Tail[i]=min(column)+Tail[i]

 #   print("Tail of machines=",Tail)

    Total_pt_and_Head_and_Tail_of_Machines=[0]*m
    for i in range(m):
        Total_pt_and_Head_and_Tail_of_Machines[i]=Total_pt_of_Machines[i]+Head[i]+Tail[i]

    Max_Total_pt_and_Head_and_Tail_of_Machines=max(Total_pt_and_Head_and_Tail_of_Machines)

    LB2=Max_Total_pt_and_Head_and_Tail_of_Machines+Total_tr_Time
 #   print("LB2=",LB2)



    Tr_Time_of_All_Activities= (2*n-q)*Total_tr_Time
  #  print("Total transportation time of all the activities=",Tr_Time_of_All_Activities)


    LB3= min(row[0] for row in pt)+ math.ceil(Tr_Time_of_All_Activities/q) + min(row[m-1] for row in pt)

  #  print("LB3=",LB3)

    print("Best LB",LB1,LB2,LB3)

    return(max(LB1,LB2,LB3))
    

# from Extract_Parameters import *

# file_path = r"C:\Users\Amir\OneDrive - uni-passau.de\RMS\Python\Generated instances/5-2 (1).txt"                   # PC
# parameters = Extract_Parameters(file_path)
# n=parameters['num_jobs']
# m=parameters['num_machines']
# pt=parameters["processing_times"]
# tt=parameters["transportation_times"]
# b_input=parameters["input_buffer"]
# b_output=parameters["output_buffer"]
# print(n,m,pt,tt,b_input,b_output)

# print(Lower_Bounds(n,m,tt,pt))