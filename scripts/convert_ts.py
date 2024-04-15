import json
from scipy.spatial.transform import Rotation as R
import numpy as np



def convert(T_CW):
    # T_CW = np.eye(4)
    # T_CW[:3, 3] = PQ_CW[:3]
    # R_CW = R.from_quat(PQ_CW[3:]).as_matrix()
    # T_CW[:3, :3] = R_CW

    P_CD = np.fromstring("0.031904; 0.0024262; -0.003753", sep='; ')
    R_CD = R.from_quat(np.fromstring("0.046186; 0.00043189; 0.0022685; 0.99893", sep='; ')).as_matrix()
    T_CD = np.eye(4)
    T_CD[:3, :3] = R_CD
    T_CD[:3, 3] = P_CD

    P_DB = np.fromstring("0; 0; 0.0018", sep='; ')
    R_DB = R.from_quat(np.fromstring("0.52548; -0.52548; 0.47315; -0.47315", sep='; ')).as_matrix()
    T_DB = np.eye(4)
    T_DB[:3, :3] = R_DB
    T_DB[:3, 3] = P_DB

    T_DC = np.linalg.inv(T_CD)
    T_BD = np.linalg.inv(T_DB)
    T_WC = np.linalg.inv(T_CW)
    T_BW = np.linalg.inv(T_DB @ T_CD @ T_WC)

    P_BW = T_BW[:3, 3]
    R_BW = T_BW[:3, :3]
    Q_BW = R.from_matrix(R_BW).as_quat()
    PQ_BW = np.concatenate((P_BW, Q_BW))

    return PQ_BW



def load_initial_calib(fname):
    json_file = open(fname, "r")
    json_str = json_file.read()
    json_data = json.loads(json_str)
    json_file.close()

    T_CWs = []
    for i in range(4):
        data = json_data[i]
        P_CW = np.asarray(data["camera_base_pos"])
        R_CW = np.asarray(data["camera_base_ori"])

        T_CW = np.eye(4)
        T_CW[:3, 3] = P_CW
        T_CW[:3, :3] = R_CW

        T_CWs.append(T_CW)

    return T_CWs



def main():
    # s0 = "0.4978142694066819 0.1558892662706566 0.5142234163337839 -0.0115054 -0.9312607 0.3639151 0.0136719"
    # s1 = "0.9053414980319217 -0.3248654104314837 0.50 -0.6671536 -0.6531739 0.2479928 0.258398"
    # s2 = "0.6012948795551849 -0.7427064134412527 0.51  -0.9236478 -0.0006818 -0.0059989 0.3831949"
    # s3 = "0.21165190037972413 -0.6043889648718954 0.51 -0.6964459 0.5486815 -0.0594016 0.4586755"

    fname = "/home/beisner/calibration.json"
    T_CWs = load_initial_calib(fname)



    # PQ_BW_0 = np.fromstring(s0, sep=' ')
    # PQ_BW_1 = np.fromstring(s1, sep=' ')

    
    # PQ_BW_2 = np.fromstring(s2, sep=' ')
    # PQ_BW_3 = np.fromstring(s3, sep=' ')


    for T_CW, i in zip(T_CWs, [0, 1, 3, 2]):
        frag = np.array2string(convert(T_CW), separator=' ')
        s = f'<node pkg="tf" type="static_transform_publisher" name="link{i}_broadcaster" args="' +  frag + f'panda_link0 {i}_camera_base 100" />'
        # print(np.array2string(convert(T_CW), separator=' '))
        print(s)
    
    # print(np.array2string(PQ_BW, separator=' '))
    # breakpoint()

if __name__ == "__main__":
    main()