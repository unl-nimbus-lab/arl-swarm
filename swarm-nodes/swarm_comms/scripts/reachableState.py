class reachableState:
    def __init__(self, node_id):
        self.node_id = node_id

        self.xmin = 0.0
        self.xmax = 0.0
        self.ymin = 0.0
        self.ymax = 0.0
        self.zmin = 0.0
        self.zmax = 0.0
        self.rs_time = 0.0 #When the reachable space is no longer valid
        self.bc_time = 0.0 #When the original message was broadcast

    def __str__(self):
        ret_str = "id:"+str(self.node_id)
        ret_str +=" x min:"+str(self.xmin)+ " x max:"+str(self.xmax)
        ret_str +=" y min:"+str(self.ymin)+ " y max:"+str(self.ymax)
        ret_str +=" z min:"+str(self.zmin)+ " z max:"+str(self.zmax)

        return ret_str

