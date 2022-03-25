# -*- coding: utf-8 -*-
# @Time    : 2021/12/7 9:55
# @Author  : Praise
# @File    : CVRP.py
# 模型参考：
import pandas as pd
from gurobipy import Model,GRB,tuplelist,tupledict,quicksum

def readXlsxFile(filename):
    df=pd.read_excel(filename,sheet_name=None)
    "读取节点信息"
    df_node=df['node']
    depot=df_node['id'][0]
    N = [depot]
    C = []
    Q = {}
    for i in range(1,df_node.shape[0]):
        id=df_node['id'][i]
        demand=df_node['demand'][i]
        N.append(id)
        C.append(id)
        Q[id]=int(demand)
    N = tuplelist(N)
    C = tuplelist(C)
    Q=tupledict(Q)

    "读取网络弧信息"
    Cost={}
    df_link=df['link']
    for i in range(df_link.shape[0]):
        from_node_id=df_link['from_node_id'][i]
        to_node_id=df_link['to_node_id'][i]
        cost=df_link['link_cost'][i]
        Cost[from_node_id,to_node_id]=cost
    Cost=tupledict(Cost)
    return depot,C,N,Q,Cost

def solveCVRPModel(depot,C,N,Q,Cost,n_vehicles=20,CAP=100):
    """
    :param depot: 车场索引
    :param C: 需求点集合
    :param N: 所有点集合
    :param Q: 需求集合
    :param Cost: 弧运输成本集合
    :param n_vehicles: 最大车辆数量
    :param CAP: 车辆容量
    :return:
    """
    "构建车队"
    K=tuplelist([f'v'+str(k) for k in range(n_vehicles)])
    "实例化模型"
    model=Model('cvrp')
    "添加变量"
    X=model.addVars( N,N,K,vtype=GRB.BINARY,name='X[i,j,k]')
    Y=model.addVars( K,N,vtype=GRB.BINARY,name='Y[k,i]')
    U=model.addVars( K,N,vtype=GRB.CONTINUOUS,name='U[k,i]')
    "目标函数：最小化路径成本"
    z1=quicksum( Cost[i,j]*X[i,j,k] for i in N for j in N for k in K if i!=j)
    model.setObjective(z1,GRB.MINIMIZE)
    "约束：需求覆盖约束"
    model.addConstrs( quicksum( Y[k,i] for k in K ) ==1 for i in C )
    "约束：车辆数量约束"
    model.addConstr( quicksum( Y[k,depot] for k in K) == n_vehicles )
    "约束：流平衡"
    model.addConstrs( quicksum( X[i,j,k] for j in N ) == quicksum( X[j,i,k] for j in N ) for i in N for k in K )
    "约束：决策变量关联"
    model.addConstrs( quicksum( X[i,j,k] for j in N ) == Y[k,i] for i in N for k in K )
    "约束：容量限制"
    model.addConstrs( quicksum( Q[i]*Y[k,i] for i in C ) <= CAP for k in K )
    "约束：破除子环"
    model.addConstrs( U[k,i]-U[k,j]+CAP*X[i,j,k] <= CAP-Q[i] for i in C for j in C for k in K )
    model.addConstrs( Q[i] <= U[k,i] for k in K for i in C )
    model.addConstrs( U[k,i] <= CAP for k in K for i in C)
    "求解"
    model.Params.TimeLimit = 300 # 设置求解时间上限
    model.optimize()
    if model.status == GRB.Status.OPTIMAL or model.status == GRB.Status.TIME_LIMIT:
        for k in K:
            for i in N:
                for j in N:
                    if i!=j :
                        if X[i,j,k].x>0:
                            print("X[{},{},{}]=1".format(i,j,k))
        print("obj:{}".format(model.objVal))
    else:
        print("no solution")


if __name__=='__main__':
    depot,C,N,Q,Cost=readXlsxFile(filename=r'D:\code\py_projects\2021-05-24-algorithms-demos\datasets\CVRP\cvrp_for_gurobi.xlsx')
    solveCVRPModel(depot=depot, C=C, N=N, Q=Q, Cost=Cost, n_vehicles=20, CAP=100)




