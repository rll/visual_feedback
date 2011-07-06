import roslib
roslib.load_manifest("patch_vision")

import cvxmod as cvx
import numpy as np

def run_opt( feature_lists, reference_indices, alpha ):
    """
    run_opt( feature_lists ) -> weights
    feature_lists is a list of I image_feature_sets
        image_feature_sets are a list of P stacked_features
        stacked_features are a list of N different feature types
    reference_indices is a set of indices which will be held out as "reference"
    performs the opt:
        min sum_{i_r in ref_idx} sum_{i < I not in ref_idx} sum_{p < P}
            w' * ||f_{i_r,p,n} - f_{i,p,n}||^2 - alpha/(P-1) * sum_{p'<p} || f_{i_r,p,n} - f_{i,p',n} ||^2
    """

    I = len( feature_lists )
    P = len( feature_lists[0] )
    N = len( feature_lists[0][0] )

    non_reference_indices = [i for i in range(I) if not i in reference_indices]
    closeness_reward = np.zeros( N )
    uniqueness_penalty = np.zeros( N )
    f = feature_lists
    for i_r in reference_indices:
        for i in non_reference_indices:
            for p in range(P):
                for n in range(N):
                    closeness_reward[ n ] += feature_distance( f[i_r][p][n], f[i][p][n] )
                    for p_false in range(p):
                        uniqueness_penalty[ n ] += feature_distance( f[i_r][p][n], f[i][p_false][n] )
    c = cvx.param('c', value = cvx.matrix( closeness_reward - alpha / float(P-1) * uniqueness_penalty ) )
    print c.value
    w = cvx.optvar('w', N )
    w.pos = True
    w | cvx.In | cvx.norm1ball(N)
    p = cvx.problem()
    p.objective = cvx.minimize( cvx.tp(c) * w  )
    p.constr = [ cvx.sum(w) == 1]
    print "Running solver"
    p.solve()
    print "Ran!"
    return np.array( w.value )


def feature_distance( vec1, vec2 ):
    diff = vec1 - vec2
    return np.sqrt(np.dot(diff,diff))

