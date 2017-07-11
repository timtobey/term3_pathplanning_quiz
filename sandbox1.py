import numpy as np


def shortest_path(npcostmap,init,goal):
    rows_to_travel = goal[0] - init[0]
    cols_to_travel = goal[1] - init[1]
    rstep = 0
    cstep = 0

    if rows_to_travel <0:
        rstep =-1
    else: rstep = 1

    if cols_to_travel <0:
        cstep =-1
    else: cstep = 1

    ir = init[0]
    ic = init[1]
    gr = goal[0]
    gc = goal[1]
    while (ir != goal[0]) or (ic != goal[1]):
        if ir != gr:
            ir = ir+ rstep
        if ic != gc:
            ic = ic + cstep
        if (ir != goal[0]) or (ic != goal[1]):
            npcostmap[ir,ic] =1
            #print(npcostmap)

def adjustpoints(point):
    point =(point[0]+1,point[1]+1)
    return point

def fillholes(npgrid,init,goal):

    rw,rc =  npgrid.shape
    npgrid[goal[0],goal[1]] = -1
    npgrid[init[0],init[1]] = -1
    break_me = True
    while break_me:
        break_me = False
        for c in range (1,rc):
            for r in range (1,rw):

                if npgrid[r,c] == 0:
                    wallvalue = npgrid[r-1,c]+npgrid[r+1,c]+ npgrid[r,c-1]+ npgrid[r,c+1]
                    if (wallvalue == 3):
                        npgrid[r,c] = 1
                        break_me = True
                r = r+ 1
            r = 1
            c = c + 1

def costmap(array,padfactor,init,goal):
    npzeros = np.array(array)
    npzeros = np.zeros((npzeros.shape[0],npzeros.shape[1]))
    npones = np.ones((npzeros.shape[0]+2,npzeros.shape[1]+2))*padfactor
    npones[1:npzeros.shape[0]+1,1:npzeros.shape[1]+1] = npzeros
    npcostmap = npones
    #npcostmap[goal[0],goal[1]] = -2
    #npcostmap[init[0],init[1]] = 100
    return npcostmap

def newgrid(array,padfactor):
    npgrid = np.array(array)
    npones = np.ones((npgrid.shape[0]+2,npgrid.shape[1]+2))*padfactor
    npones[1:npgrid.shape[0]+1,1:npgrid.shape[1]+1] = npgrid
    npgrid = npones
    return npgrid


def map_vert(gr,gc,step,npcostmap):
    if npcostmap.shape[0] > gr+step:
        while (npcostmap.shape[0] > gr+step) and ((gr+step) >= 0) :
            if npcostmap[gr+step,gc] == 0:
                npcostmap[gr+step,gc] = npcostmap[gr,gc]+1
            gr = gr + step


def map_horz(gr,gc,step,npcostmap):
    if npcostmap.shape[1] > gc+step:
        while (npcostmap.shape[1] > gc+step) and ((gc+step) >= 0) :

            if npcostmap[gr,gc+step] == 0:
                npcostmap[gr,gc+step] = npcostmap[gr,gc]+1
            gc = gc + step

def loop_vert(npcostmap):
    for c in range(1,npcostmap.shape[1]-1):
        for r in range(1,npcostmap.shape[0]-1):
            if npcostmap[r,c] != 0:
                #print("found value")
                map_vert(r,c,1,npcostmap)
                map_vert(r,c,-1,npcostmap)
                #print(npcostmap)
            r = r+1
        c = c+1
def loop_horz(npcostmap):
    for c in range(1,npcostmap.shape[1]-1):
        for r in range(1,npcostmap.shape[0]-1):
            if npcostmap[r,c] != 0:
                #print("found value")
                map_horz(r,c,1,npcostmap)
                map_horz(r,c,-1,npcostmap)
                #print(npcostmap)
            r = r+1

        c = c+1

def update_costmap(costmap,init,goal):
    shortest_path(npcostmap,init,goal)
    loop_vert(npcostmap)
    loop_horz(npcostmap)
    npcostmap[(goal[0],goal[1])]=.5
def vgrid(grid):
    npnewgrid = newgrid(grid,1)
    fillholes(npnewgrid,init,goal)
    npnewgrid = npnewgrid * -1

    npnewgrid[goal[0],goal[1]] = 1
    npnewgrid[init[0],init[1]] = 0

    npcoords = np.array([[-1,0],[1,0],[0,-1],[0,1]])
    r,c = npcoords.shape
    cursor=[1,1]
    found_box = True
    while found_box:
        found_box = False
        for c in range(1,npnewgrid.shape[1]-1):
            for r in range(1,npnewgrid.shape[0]-1):
                cell_to_test = npnewgrid[r,c]
                if cell_to_test >0:
                    for i in range(0,npcoords.shape[0]):
                        testr = r + npcoords[i,0]
                        testc = c + npcoords[i,1]
                        if npnewgrid[testr, testc] != -1 and npnewgrid[testr, testc] == 0:
                            found_box = True
                            npnewgrid[testr,testc] = 1
                r= r+1
            c =c+1

    return npnewgrid

def optimize(movemap,cursor,rdir,cdir,move,path):
    proposed_move = [cursor[0]+rdir,cursor[1]+cdir]
    if movemap[(proposed_move[0],proposed_move[1])] > path:
        path = movemap[(proposed_move[0],proposed_move[1])]
        move = proposed_move
    return move,path

def path_finder(npcostmap,npgrid,rdir,cdir,cursor,cheapest,move,start):
    proposed_move = [cursor[0]+rdir,cursor[1]+cdir]
    if proposed_move != start:
        cost= npcostmap[proposed_move[0],proposed_move[1]]
        gridvalue = npgrid[proposed_move[0],proposed_move[1]]

        if (cost < cheapest) and (gridvalue!= 1) and (cost >=0) :
            cheapest = cost
            move = [cursor[0]+rdir,cursor[1]+cdir]
    return move, cheapest


grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1




goal = adjustpoints(goal) # reset goal to new place after padding
init = adjustpoints(init)# reset init to new place after padding

npnewgrid = newgrid(grid,1)
fillholes(npnewgrid,init,goal)
vgrid = vgrid(grid)
if vgrid[(init[0],init[1])] == 0:
    print("fail")
    exit()

npcostmap = costmap(grid,-1,init,goal)
update_costmap(costmap,init,goal)



cursor = init
cheapest =100
start =[0,0]
move =[0,0]
cost = 1000
npcoords = np.array([[-1,0],[1,0],[0,-1],[0,1]])
r,c = npcoords.shape
movecount =0
npmovemap = np.zeros((npcostmap.shape[0],npcostmap.shape[1]))
npmovemap[(cursor[0],cursor[1])] =1
while (cursor[0] != goal[0]) or (cursor[1]!= goal[1]):
    for i in range (0, r):
        rdir= npcoords[i,0]
        cdir =npcoords[i,1]
        move, cheapest =  path_finder(npcostmap,npnewgrid,rdir,cdir,cursor,cheapest,move,start)
    npmovemap[(cursor[0],cursor[1])] =movecount
    start = cursor
    cheapest =100

    cursor = move
    movecount = movecount +1
    #print("start point",start,"moved to:",cursor,"Number of Moves: ",movecount)
    npcostmap = costmap(grid,-1,cursor,goal)
    update_costmap(costmap,cursor,goal)

npmovemap[(cursor[0],cursor[1])] =movecount

### optimize
path = 0
movecount =1
move =[-1,-1]
cursor = init
while (cursor[0] != goal[0]) or (cursor[1]!= goal[1]):
    for i in range (0, r):
        rdir= npcoords[i,0]
        cdir =npcoords[i,1]
        move,path = optimize(npmovemap,cursor,rdir,cdir,move,path)
    path =0
    if move[0]==-1:
        print("no move found")
        break
    cursor = move
    npmovemap[(cursor[0],cursor[1])] = movecount

    lastmovecount = movecount
    movecount = movecount +1
    move =[-1,-1]
path = [cursor[0],cursor[1],lastmovecount]
print(npmovemap)

