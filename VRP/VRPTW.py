# -*- coding: utf-8 -*-
# @Time    : 2021/12/8 9:01
# @Author  : Praise
# @File    : VRPTW.py
# obj:
import csv
from gurobipy import GRB,Model,quicksum,tupledict,tuplelist

def readCsvFile(node_file,link_file):
    """
    :param node_file: 节点文件，
    :param link_file: 网络弧文件
    :return:
    """
    N=[] #所有节点
    Q={} #节点需求
    TT={} #节点旅行时间
    ET={} #节点最早开始服务时间
    LT={} #节点最晚结束服务时间
    ST={} #节点服务时间
    Cost={}
    with open(node_file,'r') as f:
        node_reader=csv.DictReader(f)
        for row in node_reader:
            node_id = row['id']
            demand = float(row['demand'])
            start_time = float(row['start_time'])
            end_time = float(row['end_time'])
            service_time = float(row['service_time'])
            N.append(node_id)
            Q[node_id] = demand
            ET[node_id] = start_time
            LT[node_id] = end_time
            ST[node_id] = service_time
    with open(link_file,'r') as f:
        link_reader = csv.DictReader(f)
        for row in link_reader:
            from_node_id = row['from_node_id']
            to_node_id = row['to_node_id']
            travel_time = float(row['travel_time'])
            travel_cost = float(row['link_cost'])
            TT[from_node_id,to_node_id] = travel_time
            Cost[from_node_id,to_node_id] = travel_cost
    return N,Q,TT,ET,LT,ST,Cost

def saveFile(data):
    outfile = open('results.csv', 'w', newline='', errors='ignore')
    write = csv.writer(outfile)
    write.writerow(['from_node_id', 'to_node_id', 'vehicle','Ti','Tj'])
    for v in data:
        write.writerow(v)
    outfile.close()

def solveVRPTWModel(N,Q,TT,ET,LT,ST,Cost,CAP,K):
    """
    :param N: 所有节点集合，其中N[0]为车场
    :param Q: 节点需求集合
    :param TT: 旅行时间
    :param ET: 节点最早开始服务时间
    :param LT：节点最晚结束服务时间
    :param ST: 节点服务时间
    :param CAP: 车辆容量
    :param Cost: 旅行费用
    :param K: 车队数量
    :return:
    """
    C=tuplelist(N[1:]) #需求节点
    N=tuplelist(N)
    Q=tupledict(Q)
    TT=tupledict(TT)
    E=tupledict(ET)
    L=tupledict(LT)
    S=tupledict(ST)
    K=tuplelist([f'v'+str(i) for i in range(K)])
    M=10**5
    depot = N[0]
    "创建模型"
    model=Model()
    "添加变量"
    X=model.addVars(N,N,K,vtype=GRB.BINARY,name='X(i,j,k)')
    T=model.addVars( N,K,vtype=GRB.CONTINUOUS,lb=0,name='T[i,k]')
    "设置目标函数"
    z1=quicksum( Cost[i,j]*X[i,j,k] for i in N for j in N for k in K if i!=j)
    model.setObjective(z1,GRB.MINIMIZE)
    "车辆起点约束"
    model.addConstrs(quicksum(X[depot, j, k] for j in N) == 1 for k in K)
    "车辆路径连续约束"
    model.addConstrs( quicksum(X[i,j,k] for j in N if j!=i)==quicksum(X[j,i,k] for j in N if j!=i) for i in C for k in K)
    "车辆终点约束"
    model.addConstrs(quicksum(X[j, depot, k] for j in N) == 1 for k in K)
    "需求服务约束"
    model.addConstrs( quicksum(X[i,j,k] for k in K for j in N if j!=i)==1 for i in C)
    "车辆容量约束"
    model.addConstrs( quicksum(Q[i]*X[i,j,k] for i in C for j in N if i!=j)<=CAP for k in K )
    "时间窗约束"
    model.addConstrs( T[i,k]+S[i]+TT[i,j]-(1-X[i,j,k])*M<=T[j,k] for i in C for j in C for k in K if i!=j )
    model.addConstrs( T[i,k] >= E[i] for i in N for k in K)
    model.addConstrs( T[i,k] <= L[i] for i in N for k in K)
    "设置模型参数"
    model.Params.TimeLimit = 300 # 规模较大时可设置求解时间限制
    # model.setParam('OutputFlag', 0) #是否输出求解日志
    "模型求解"
    model.optimize()
    "判断求解状态"
    if model.status == GRB.Status.OPTIMAL or model.status == GRB.Status.TIME_LIMIT:
        print('obj={}'.format(model.objVal))
        res=[]
        for k in K:
            for i in N:
                for j in N:
                    if i!=j:
                        if X[i,j,k].x>0:
                            print("X[{},{},{}]=1".format(i,j,k))
                            res.append([i,j,k,T[i,k].x,T[j,k].x])
        saveFile(res)
    else:
        print("no solution")

if __name__=='__main__':
    node_file=r'D:\code\py_projects\2021-05-24-algorithms-demos\datasets\MDVRPTW\node_for_gurobi.csv'
    link_file=r'D:\code\py_projects\2021-05-24-algorithms-demos\datasets\MDVRPTW\link_for_gurobi.csv'
    N,Q,TT,ET,LT,ST,Cost=readCsvFile(node_file,link_file)
    solveVRPTWModel(N=N,Q=Q,TT=TT,ET=ET,LT=LT,ST=ST,Cost=Cost,CAP=80,K=15)
