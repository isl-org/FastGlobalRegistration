#!/usr/bin/python

# ----------------------------------------------------------------------------
# -                       Fast Global Registration                           -
# ----------------------------------------------------------------------------
# The MIT License (MIT)
#
# Copyright (c) Intel Corporation 2016
# Qianyi Zhou <Qianyi.Zhou@gmail.com>
# Jaesik Park <syncle@gmail.com>
# Vladlen Koltun <vkoltun@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.
# ----------------------------------------------------------------------------

import sys
import numpy as np
import numpy.linalg

class CameraPose:
    def __init__(self, meta, mat):
        self.metadata = meta
        self.pose = mat
    def __str__(self):
        return 'Metadata : ' + ' '.join(map(str, self.metadata)) + '\n' + \
            "Pose : " + "\n" + np.array_str(self.pose)


def read_trajectory(filename):
    traj = []
    with open(filename, 'r') as f:
        metastr = f.readline()
        while metastr:
            metadata = map(int, metastr.split())
            mat = np.zeros(shape = (4, 4))
            for i in range(4):
                matstr = f.readline()
                mat[i, :] = np.fromstring(matstr, dtype = float, sep=' \t')
            traj.append(CameraPose(metadata, mat))
            metastr = f.readline()
    return traj

def write_trajectory(traj, filename):
    with open(filename, 'w') as f:
        for x in traj:
            p = x.pose.tolist()
            f.write(' '.join(map(str, x.metadata)) + '\n')
            f.write('\n'.join(' '.join(map('{0:.12f}'.format, p[i])) for i in range(4)))
            f.write('\n')

class TransformationInformation:
    def __init__(self, meta, mat):
        self.metadata = meta
        self.info = mat
    def __str__(self):
        return 'Metadata : ' + ' '.join(map(str, self.metadata)) + '\n' + \
            "Information : " + "\n" + np.array_str(self.info)


def read_information(filename):
    info = []
    with open(filename, 'r') as f:
        metastr = f.readline()
        while metastr:
            metadata = map(int, metastr.split())
            mat = np.zeros(shape = (6, 6))
            for i in range(6):
                matstr = f.readline()
                mat[i, :] = np.fromstring(matstr, dtype = float, sep=' \t')
            info.append(TransformationInformation(metadata, mat))
            metastr = f.readline()
    return info

def write_information(info, filename):
    with open(filename, 'w') as f:
        for x in info:
            p = x.info.tolist()
            f.write(' '.join(map(str, x.metadata)) + '\n')
            f.write('\n'.join(' '.join(map('{0:.12f}'.format, p[i])) for i in range(6)))
            f.write('\n')

def dcm2quat(DCM):
    # this is consistent with the matlab function in
    # the Aerospace Toolbox
    qout = np.zeros(shape = (4, 1));
    qout[0] = 0.5 * np.sqrt(1 + DCM[0,0] + DCM[1,1] + DCM[2,2]);
    qout[1] = - (DCM[2,1] - DCM[1,2]) / ( 4 * qout[0] );
    qout[2] = - (DCM[0,2] - DCM[2,0]) / ( 4 * qout[0] );
    qout[3] = - (DCM[1,0] - DCM[0,1]) / ( 4 * qout[0] );
    return qout

def get_transformation_error(trans, info):
    te = trans[0:3, 3:4]
    qt = dcm2quat(trans[0:3, 0:3])
    er = numpy.concatenate([te, -qt[1:4]])
    return np.dot(np.dot(np.transpose(er), info), er) / info[0,0]

def main(argv):
    if (len(argv) < 3):
        print('Usage:')
        print('    > python evaluate.py <trajectory.log> <gt.log> <gt.info>')
        print('')
        print('Example:')
        print('    > python evaluate.py ../../dataset/pairwise_noise_xyz_level_01_01_rot_05/output.txt ../../dataset/pairwise_noise_xyz_level_01_01_rot_05/gt.log ../../dataset/pairwise_noise_xyz_level_01_01_rot_05/gt.info')
        print('')
        sys.exit()
    traj = read_trajectory(argv[0])
    gt_traj = read_trajectory(argv[1])
    gt_info = read_information(argv[2])
    err = get_transformation_error(np.dot(np.linalg.inv(gt_traj[0].pose), traj[0].pose), gt_info[0].info)
    print('Error is ' + str(err[0,0]))

if __name__ == "__main__":
    main(sys.argv[1:])
