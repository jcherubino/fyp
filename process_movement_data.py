'''
Simple script to process movement data error
'''
import argparse
import numpy as np 
import matplotlib.pyplot as plt

def interpolate_path_points(path_waypoints, steps_per_segment):
    '''
    Generate numpy array of x and y points between each path waypoint with a 
    steps_per_segment
    '''
    # create array to store values
    path = np.empty(shape=(steps_per_segment*(path_waypoints.shape[0]-1), 2))

    for i in range(0, path_waypoints.shape[0]-1):
        i_next = i+1
        # x segments the same. therefore interpolate along y
        if path_waypoints[i][0] == path_waypoints[i_next][0]:
            y_segment = np.linspace(path_waypoints[i][1], path_waypoints[i_next][1], num=steps_per_segment)
            # also must save repeated values
            x_segment = np.repeat(path_waypoints[i][0], steps_per_segment)
            
        # x segments different, interpolate along x
        else:
            x_segment = np.linspace(path_waypoints[i][0], path_waypoints[i_next][0], num=steps_per_segment)
            # also must save repeated values
            y_segment = np.repeat(path_waypoints[i][1], steps_per_segment)

        path[i*steps_per_segment:i_next*steps_per_segment,0] = x_segment
        path[i*steps_per_segment:i_next*steps_per_segment,1] = y_segment

    return path

def show_path(path, raw, fused):
    '''
    Visualise all data
    '''
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)
    
    ax.scatter(path[:,0], path[:,1], label='True path')
    ax.scatter(raw[:,0], raw[:,1], label='Raw UWB',)
    ax.scatter(fused[:,0], fused[:,1], label='Fused')
    ax.legend()
    
    plt.show()

def compute_min_distance_to_path(path, point):
    '''
    Compute the minimum distance of a point to the path
    '''
    dist = np.inf
    for pos in path:
        cur_dist = np.linalg.norm(pos - point)
        if cur_dist < dist:
            dist = cur_dist
    return dist

def compute_error(path, measurements):
    '''
    Compute error of x,y measurements against the path

    Achieve by computing the distance to every point on the path. Minimum distance
    to the path is then the distance for that x,y measurement. Compute error for 
    all measurements and average
    '''
    cumulative_error = 0
    for measurement in measurements:
        dist_error = compute_min_distance_to_path(path, measurement)
        cumulative_error += dist_error

    avg_error = cumulative_error/measurements.shape[0]
    return avg_error

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('raw', help='Filepath of csv containing raw UWB measurements')
    parser.add_argument('fused', help='Filepath of csv containing fused pose measurements')
    parser.add_argument('path', help='Filepath of csv containing true path positions travelled')
    parser.add_argument('--enc', help='Encoding type', default='utf-8-sig', type=str)
    parser.add_argument('--segsteps', help='Number of steps per segment when interpolating path', default=200, type=int)


    args = parser.parse_args()

    raw = np.loadtxt(args.raw, delimiter=',', encoding=args.enc)
    fused = np.loadtxt(args.fused, delimiter=',', encoding=args.enc)
    path_waypoints = np.loadtxt(args.path, delimiter=',', encoding=args.enc)

    path = interpolate_path_points(path_waypoints, args.segsteps)

    show_path(path, raw, fused)

    raw_error = compute_error(path, raw)
    fused_error = compute_error(path, fused)

    print(f"Raw error: {raw_error}")
    print(f"Fused error: {fused_error}")
