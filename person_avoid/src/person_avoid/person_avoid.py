#! /usr/bin/python

import numpy as np
from trajectory_gen.spline_traj_executor import SplineTraj

def avoid_traj(xi, xf, distmove, midptfrac1, midptfrac2, wayptvel, 
               qd_i=None, qd_f=None, qdd_i=None, qdd_f=None):
    diff = xf-xi
    normdiff = np.linalg.norm(diff)
    unitdiff = diff/normdiff
     # find 2 middle points to move away
    midpt1 = xi + midptfrac1 * diff
    midpt2 = xi + midptfrac2 * diff
    orthdiff = np.array([unitdiff[1], -unitdiff[0]]) # orthogonal unit vector
    if np.sum(midpt1*orthdiff) > 0:
        # if it's pointing away from the origin, point it back the other direction
        orthdiff = -orthdiff

    # move the middle line segement orthogonally away from the straight line
    wpt1 = midpt1 + orthdiff*distmove
    wpt2 = midpt2 + orthdiff*distmove
    q = np.array([xi, wpt1, wpt2, xf]) # knots
    
    # simple heuristic: use average velocity to find times
    if qd_i is None:
        initvel = wayptvel
    else:
        initvel = np.linalg.norm(qd_i)
    time1 = np.linalg.norm(wpt1-xi)/initvel
    time2 = np.linalg.norm(wpt2-wpt1)/wayptvel
    time3 = np.linalg.norm(xf-wpt2)/wayptvel
    tknots = np.cumsum([0., time1, time2, time3])

    st = SplineTraj.generate(tknots,q,qd_i=qd_i,qd_f=qd_f,qdd_i=qdd_i,qdd_f=qdd_f)
    return st, q

def main():
    import matplotlib.pyplot as plt
    fig = plt.figure()
    xi0 = np.array([0.8, 0.0])
    xf = np.array([0.0, -0.8])
    midptfrac1 = 0.5
    midptfrac2 = 0.8
    wayptvels = [0.3, 0.10]

    st, qknots = avoid_traj(xi0, xf, 0.1, midptfrac1, midptfrac2, wayptvels[0])
    qmid, qdmid, qddmid = st.sample(2.7)
    print qmid, qdmid, qddmid
    #qd_is = [None, None]
    qd_is = [None, qdmid]
    #qdd_is = [None, None]
    qdd_is = [None, qddmid]

    xis = [xi0, qmid]
    # tknots = [0., 4., 8., 12.]
    # q = [[0.8, 0.0], [0.4, -0.2], [0.2, -0.4], [0.0, -0.8]]

    for trial in range(2):
        xi = xis[trial]
        wayptvel = wayptvels[trial]
        qd_i = qd_is[trial]
        qdd_i = qdd_is[trial]
        for distmove in [-0.1, 0.0, 0.1, 0.2, 0.3]:
            st, qknots = avoid_traj(xi, xf, distmove, midptfrac1, midptfrac2, wayptvel,
                                    qd_i=qd_i, qdd_i=qdd_i)
            ts = np.linspace(0., st.duration, np.round(st.duration*5))
            qsamp = np.array([st.sample(t)[0] for t in ts])

            plt.plot(qsamp[:,0],qsamp[:,1],'.')
            plt.plot(qknots[:,0],qknots[:,1],'xr')

    plt.show()

if __name__ == "__main__":
    main()
