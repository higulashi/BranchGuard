# References

- Shenghua Xu et al. **RCLane: Relay Chain Prediction for Lane Detection**, ECCV 2022. [Paper](https://arxiv.org/abs/2207.09399). Studies lane reconstruction under branching and other complex line shapes. BranchGuard uses fixed scan rows and beam search rather than a learned segmentation and relay network.
- Namdar Homayounfar et al. **DAGMapper: Learning to Map by Discovering Lane Topology**, ICCV 2019. [Paper](https://arxiv.org/abs/2012.12377). Models highway lane boundaries and forks/merges as a directed graph. The present project performs local guide-line selection, without HD-map generation.
- R. Craig Coulter. **Implementation of the Pure Pursuit Path Tracking Algorithm**, Carnegie Mellon University, 1992. [Technical report](https://www.ri.cmu.edu/publications/implementation-of-the-pure-pursuit-path-tracking-algorithm/). The common steering formula in the simulator follows this geometric principle.

Beam search, bounded candidate sets, inter-frame geometric association and persistence checks are established techniques. This repository explores their combination for a small grayscale guide-line testbed, with emphasis on motion-corrected near-path association and a confirmation rule that leaves far-path branching free. The implementation and synthetic experiments do not establish research priority.

No third-party algorithm source files are vendored. NumPy and Pillow provide simulation and rendering utilities; Zig is used as a C compiler when available. Their respective licenses apply to those dependencies. Repository code is released under MIT.
