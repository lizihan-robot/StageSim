import numpy as np
import json 
import os

mark_name = "Plane"
current_path = os.path.abspath(__file__)
parent_directory = os.path.dirname(os.path.dirname(current_path))
file_path_name = parent_directory+"/maps/plane_matrix.json"

class Map():
    def __init__(self, path = file_path_name):
        self.path = path
        _mark_matrix = self._load_plane_matrix()
        self.mark_matrix = np.array(_mark_matrix)
        
    @classmethod
    def creat_map_matrix(self, length=60, width=60, interval=1.8,offs_x=0,offs_y=0):
        x = int(length/interval)
        y = int(width/interval)
        x_coords = np.arange(offs_x+0, offs_x+x*interval, interval)
        y_coords = np.arange(offs_y+0, offs_y+y*interval, interval)
        matrix = np.zeros((x, y, 2))
        for i in range(x):
            for j in range(y):
                matrix[i][j] = [x_coords[i], y_coords[j]]
        rounded_matrix = np.around(matrix, decimals=3)
        m = Map()
        js = {"plane_matrix": rounded_matrix.tolist()}
        with open(file_path_name, "w") as f:
            json.dump(js, f)
        return m

    def _load_plane_matrix(self, path=file_path_name):
        with open(path, "r") as f:
            data = json.load(f)
        return data["plane_matrix"]

    def get_plane_pose(self, mark_name):
        # print("[MAP]: get mark pose--{} ".format(mark_name))
        i = int(mark_name[mark_name.index('e')+1:mark_name.index('_')])
        j = int(mark_name[mark_name.index('_')+1:])
        return self.mark_matrix[i, j]
    
if __name__=="__main__":
    Map().creat_map_matrix()