#!/usr/bin/env python
# coding=utf8
# calculate relative error for odometry

import numpy as np
from evo.tools import file_interface
from evo.core import sync
from evo.core import filters
from evo.core import geometry
from evo.core import metrics
from evo.core import lie_algebra as lie


class Args(object):
    def __init__(self):
        self.subcommand = "tum"
        self.t_max_diff = 0.01
        self.t_offset = 0.0
        self.ref_file = "/media/2T/fast_lio2/utbm/groundtruth/tum/utbm_robocar_dataset_20190418_roundabout_noimage.splev.tum"
        self.est_file = "/home/idriver/Documents/faster_lio_ws/src/fast_lio2/Log/faster_lio/utbm_robocar_dataset_20190418_roundabout_noimage.traj.log"
        self.delta = 100
        self.delta_unit = metrics.Unit.meters
        self.rel_delta_tol = 0.1
        self.all_pairs = False


def rpe_odom(args):
    traj_ref = file_interface.read_tum_trajectory_file(args.ref_file)
    traj_est = file_interface.read_tum_trajectory_file(args.est_file)
    ref_name, est_name = args.ref_file, args.est_file
    print("Read %d pose from ref_file: %s" % (traj_ref.num_poses, ref_name))
    print("Read %d pose from est_file: %s" % (traj_est.num_poses, est_name))

    traj_ref, traj_est = sync.associate_trajectories(
        traj_ref, traj_est, args.t_max_diff, args.t_offset,
        first_name=ref_name, snd_name=est_name)
    print("Synced poses of ref: %d, Synced poses of est: %d" % (traj_ref.num_poses, traj_est.num_poses))

    id_pairs = filters.id_pairs_from_delta(
        traj_est.poses_se3, args.delta, args.delta_unit,
        args.rel_delta_tol, all_pairs=args.all_pairs)
    print("Select %d paris from synced poses for every %d %s" % (len(id_pairs), args.delta, args.delta_unit))

    def rpe_odom_segment(ref_xyz, est_xyz, i, j):
        # align
        r_a, t_a, s = geometry.umeyama_alignment(est_xyz[i:j + 1, :].T, ref_xyz[i:j + 1, :].T, False)
        ref_d = ref_xyz[j] - ref_xyz[i]
        est_d = est_xyz[j] - est_xyz[i]
        est_d_aligned = (np.dot(r_a, est_d))
        error = np.linalg.norm(ref_d - est_d_aligned)
        return error

    errors = [
        rpe_odom_segment(traj_ref.positions_xyz, traj_est.positions_xyz, i, j)
        for i, j in id_pairs
    ]

    # rmse of errors
    rmse = np.sqrt(np.mean(np.power(errors, 2)))
    max_v = np.max(errors)
    print("relative translation error:")
    print("\trmse: %f\n\tmax: %f\n\tmin: %f\n\tmean: %f\n" % (rmse, np.max(errors), np.min(errors), np.mean(errors)))
    return rmse


if __name__ == "__main__":
    import sys

    if len(sys.argv) == 3:
        args = Args()
        args.ref_file, args.est_file = sys.argv[1], sys.argv[2]
        rmse = rpe_odom(args)
    else:
        print("Usage: exec traj_ref_file traj_est_file")
