import numpy as np

def l2_dist(v1, v2):
    v1_arr = np.array(v1)
    v2_arr = np.array(v2)
    diff = v1_arr - v2_arr
    return np.dot(diff,diff)

def chi2_dist(v1, v2):
    v1_arr = np.array(v1)
    v2_arr = np.array(v2)
    abs_sum = abs(v1_arr) + abs(v2_arr)
    diff = (v1_arr - v2_arr)**2 / abs_sum
    #Weed out nans
    for i,is_nan in enumerate(np.isnan(diff)):
        if is_nan:
            diff[i] = 0
    dist = np.dot(diff,diff)
    return dist

def get_rect_vertices(center, width, height):
    x = center[0] - (width + 1)/2.0
    y = center[1] - (height + 1)/2.0
    return (int(x),int(y)),(int(x+width),int(y+height))

def compute_knn( comparisons, key, n ):
    knn = []
    for i in range(len(comparisons)):
        dist = key( comparisons[i] )
        if len(knn) < n:
            knn.append( comparisons[i] )
        elif dist < key( knn[n-1] ):
            knn[n-1] = comparisons[i]
        else:
            continue
        knn.sort(key = key)
    return knn    
