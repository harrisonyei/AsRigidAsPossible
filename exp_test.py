from gpmm.bfm09_tf_uv import BFM_TF

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
import sys
from shutil import copyfile

# tf
import numpy as np
import tensorflow as tf

import cv2
import trimesh

# path
_curr_path = os.path.abspath(__file__) # /home/..../face
_cur_dir = os.path.dirname(_curr_path) # ./

flags = tf.app.flags

#
flags.DEFINE_string("input_cam", "output_cam.txt", "Dataset directory")
flags.DEFINE_string("input_vid", "output_vid.txt", "Dataset directory")

flags.DEFINE_string("output_dir", "data/output/", "Output directory")
flags.DEFINE_string("ckpt_file", "model/model-400000", "checkpoint file")
#flags.DEFINE_string("ckpt_file", "/home/jiaxiangshang/Downloads/202008/70_31_warpdepthepi_reg/model-400000", "checkpoint file")

#
flags.DEFINE_integer("batch_size", 1, "The size of of a sample batch")
flags.DEFINE_integer("img_width", 224, "Image(square) size")
flags.DEFINE_integer("img_height", 224, "Image(square) size")

# gpmm
flags.DEFINE_string("path_gpmm", "model/bfm09_trim_exp_uv_presplit.h5", "Dataset directory")
flags.DEFINE_integer("light_rank", 27, "3DMM coeffient rank")
flags.DEFINE_integer("gpmm_rank", 80, "3DMM coeffient rank")
flags.DEFINE_integer("gpmm_exp_rank", 64, "3DMM coeffient rank")

FLAGS = flags.FLAGS

def main():
    FLAGS.input_cam = os.path.join(_cur_dir, FLAGS.input_cam)
    FLAGS.input_vid = os.path.join(_cur_dir, FLAGS.input_vid)

    FLAGS.output_dir = os.path.join(_cur_dir, FLAGS.output_dir)

    FLAGS.ckpt_file = os.path.join(_cur_dir, FLAGS.ckpt_file)
    FLAGS.path_gpmm = os.path.join(_cur_dir, FLAGS.path_gpmm)
    
    
    if not os.path.exists(FLAGS.input_cam) or not os.path.exists(FLAGS.input_vid):
        print("Error: no dataset_dir found")

    if not os.path.exists(FLAGS.output_dir):
        os.makedirs(FLAGS.output_dir)

    h_lrgp = BFM_TF(FLAGS.path_gpmm, FLAGS.gpmm_rank, FLAGS.gpmm_exp_rank, FLAGS.batch_size, full=1)

    ## open pre stored coefficient datas
#region camera input
    f_cam = open(FLAGS.input_cam, "r")
    pts_cam = []
    exp_cam = []

    l = f_cam.readline()
    args = l.split()
    rank = int(args[0])
    exp_rank = int(args[1])

    EOF = False
    while not EOF:
        pts_coeffs = []
        for i in range(rank):
            coeff = f_cam.readline()
            if len(coeff) == 0:
                EOF = True
                break

            pts_coeffs.append(float(coeff))

        pts_cam.append(pts_coeffs)

        exp_coeffs = []
        for i in range(exp_rank):
            coeff = f_cam.readline()
            if len(coeff) == 0:
                EOF = True
                break

            exp_coeffs.append(float(coeff))
            
        exp_cam.append(exp_coeffs)
#endregion

#region video input
    f_vid = open(FLAGS.input_vid, "r")
    pts_vid = []
    exp_vid = []

    l = f_vid.readline()
    args = l.split()
    rank = int(args[0])
    exp_rank = int(args[1])

    EOF = False
    while not EOF:
        pts_coeffs = []
        for i in range(rank):
            coeff = f_vid.readline()
            if len(coeff) == 0:
                EOF = True
                break

            pts_coeffs.append(float(coeff))

        pts_vid.append(pts_coeffs)

        exp_coeffs = []
        for i in range(exp_rank):
            coeff = f_vid.readline()
            if len(coeff) == 0:
                EOF = True
                break

            exp_coeffs.append(float(coeff))
            
        exp_vid.append(exp_coeffs)
#endregion

    ## get 3DMM basis
    h_curr = h_lrgp.h_curr

    mesh_tri = h_curr.mesh_tri_np

    pt_mean = h_curr.pt_mean_np
    pt_pcaBasis = h_curr.pt_pcaBasis_np[:, :rank]
    pt_pcaStd = np.sqrt(h_curr.pt_pcaVariance_np[:rank].reshape((rank, 1)))

    exp_pcaBasis = h_curr.exp_pcaBasis_np[:, :exp_rank]
    exp_pcaStd = np.sqrt(h_curr.exp_pcaVariance_np[:exp_rank].reshape((exp_rank, 1)))

    #for i in range(len(pts_cam)):
    # 0 ,800
    i = 815
    i_target = 0
    if True:
        coeff_pts = np.array(pts_cam[i]).reshape((rank, 1))
        coeff_exp = np.array(exp_cam[i]).reshape((exp_rank, 1))

        """
        Vertex Source
        """
        coeff_var = coeff_pts * pt_pcaStd

        source_mesh_diff = np.matmul(pt_pcaBasis, coeff_var)
        source_mesh_diff = np.transpose(source_mesh_diff)

        """
        Exp Source
        """
        coeff_var = coeff_exp * exp_pcaStd

        source_exp_diff = np.matmul(exp_pcaBasis, coeff_var)
        source_exp_diff = np.transpose(source_exp_diff)

        # source mesh
        s_n = (pt_mean + source_mesh_diff).reshape(-1, 3)            # neutral
        s_e = (pt_mean + source_mesh_diff + source_exp_diff).reshape(-1, 3) # exprssion

        coeff_pts = np.array(pts_cam[i_target]).reshape((rank, 1))
        coeff_pts[1] = -3
        coeff_pts[2] = 5
        coeff_pts[3] = 4
        coeff_pts[4] = 3
        coeff_pts[5] = -2.5
        """
        Vertex target
        """
        coeff_var = coeff_pts * pt_pcaStd
        target_mesh_diff = np.matmul(pt_pcaBasis, coeff_var)
        target_mesh_diff = np.transpose(target_mesh_diff)

        # target mesh
        t_n = (pt_mean + target_mesh_diff).reshape(-1, 3)            # neutral

        ## Create expression transfer matrices A * exp_coeff = b
        n_faces = mesh_tri.shape[0]

        # A mat
        A = np.ndarray((n_faces * 2 * 3, exp_rank),dtype=np.float64) # faces * 2(edges) * 3(xyz)
        for f_idx in range(n_faces):
            face = mesh_tri[f_idx]
            # get vertex indices
            i0 = face[0]
            i1 = face[1]
            i2 = face[2]
            #exp_pcaBasis (n_vertices, exp_rank)
            # vec0
            A[f_idx * 6 + 0] = exp_pcaBasis[i1 * 3] - exp_pcaBasis[i0 * 3]
            A[f_idx * 6 + 1] = exp_pcaBasis[i1 * 3 + 1] - exp_pcaBasis[i0 * 3 + 1]
            A[f_idx * 6 + 2] = exp_pcaBasis[i1 * 3 + 2] - exp_pcaBasis[i0 * 3 + 2]
            # vec1
            A[f_idx * 6 + 3] = exp_pcaBasis[i2 * 3] - exp_pcaBasis[i0 * 3]
            A[f_idx * 6 + 4] = exp_pcaBasis[i2 * 3 + 1] - exp_pcaBasis[i0 * 3 + 1]
            A[f_idx * 6 + 5] = exp_pcaBasis[i2 * 3 + 2] - exp_pcaBasis[i0 * 3 + 2]

        # b mat
        b = np.ndarray((n_faces * 2 * 3, 1),dtype=np.float64)
        for f_idx in range(n_faces):
            face = mesh_tri[f_idx]
            # get vertex indices
            i0 = face[0]
            i1 = face[1]
            i2 = face[2]

            # local transformation Ai
            Ai = np.ndarray((3,3),dtype=np.float64)
            dn = np.ndarray((2,3),dtype=np.float64)
            de = np.ndarray((2,3),dtype=np.float64)

            dn[0] = s_n[i1] - s_n[i0]
            dn[1] = s_n[i2] - s_n[i0]
            de[0] = s_e[i1] - s_e[i0]
            de[1] = s_e[i2] - s_e[i0]

            dnTdn = np.matmul(np.transpose(dn), dn)
            det = np.linalg.det(dnTdn)
            if det != 0:
                inv = np.linalg.inv(dnTdn)
                Ai = np.matmul(inv, np.matmul(np.transpose(dn), de))
            else:
                Ai = np.identity(3)

            t_dn = np.ndarray((2, 3))
            t_dn[0] = t_n[i1] - t_n[i0]
            t_dn[1] = t_n[i2] - t_n[i0]

            t_dn = np.matmul(t_dn, Ai) - t_dn

            b[f_idx * 6 + 0] = t_dn[0,0]
            b[f_idx * 6 + 1] = t_dn[0,1]
            b[f_idx * 6 + 2] = t_dn[0,2]
            b[f_idx * 6 + 3] = t_dn[1,0]
            b[f_idx * 6 + 4] = t_dn[1,1]
            b[f_idx * 6 + 5] = t_dn[1,2]

        # pseudo inverse of matrix A_p => (At * A)^-1 * A
        u, s, vh = np.linalg.svd(A, full_matrices=False)
        smat = np.diag(s)
        A_p = np.matmul(np.transpose(vh), np.matmul(np.linalg.inv(smat), np.transpose(u)))

        # la = np.matmul(np.transpose(A), A)
        # lb = np.matmul(np.transpose(A), b)
        # new_exp = np.linalg.solve(la, lb)

        new_exp = np.matmul(A_p, b) # pseudo inv runs 1.3x faster than linalg.solve

        new_exp_diff = np.matmul(exp_pcaBasis, new_exp)
        new_exp_diff = np.transpose(new_exp_diff)

        t_e = (pt_mean + target_mesh_diff + new_exp_diff).reshape(-1, 3) # exprssion tranfer by solving quadrics

        def instanciate_mesh(name, vertices, export=True):
            ## instanciate mesh
            tri_mesh = trimesh.Trimesh(
                    vertices,
                    mesh_tri,
                    #vertex_colors=rgb_mean_3d,
                    process = False
                )
            if export:
                tri_mesh.visual.kind == 'vertex'
                path_mesh_save = os.path.join(FLAGS.output_dir, name + ".ply")
                tri_mesh.export(path_mesh_save)
            return tri_mesh

        instanciate_mesh("target_neu", t_n)
        instanciate_mesh("target_exp", t_e)

        t_e = (pt_mean + target_mesh_diff + source_exp_diff).reshape(-1, 3) # exprssion directly replace coefficients
        instanciate_mesh("target_rlp", t_e)

        instanciate_mesh("source_neu", s_n)
        instanciate_mesh("source_exp", s_e)

    print("DONE")

if __name__ == '__main__':
    main()