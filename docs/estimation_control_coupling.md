# Estimation-Control Coupling: Why the Linear KF Cannot Spiral, and When an Estimator Can

This note answers one question raised by the [ledge-drop test](results/upkie_ledge_drop.md):
a sudden drop gives the base-height estimate a transient, and the MPC plans from that wrong estimate; can a wrong estimate make a worse plan that makes a worse estimate, until the loop diverges?
It relates the question to the classic loss of robustness of LQG control, shows why the [linear KF](wheeled_biped_state_estimation.md) used by Upkie and H2 is structurally protected from it, where the coupling can still enter, and whether estimators built on a dynamics model fall back into it.
Written 2026-11-15; the paper descriptions come from their abstracts and summaries, and the risk assessments are this note's own analysis, not claims of the papers.


## 1. LQG and "there are none"

LQR alone has guaranteed robustness at the plant input: at least 6 dB of gain margin and 60 degrees of phase margin (Safonov and Athans, 1977).
[Doyle (1978), *Guaranteed Margins for LQG Regulators*](https://doyle.caltech.edu/images/doyle/6/62/GuaranteedMarginsLQGRegs.pdf), answered whether LQG keeps them with a counterexample: "there are none".
A two-state, single-input LQG design can have an arbitrarily small gain margin, even for open-loop stable, minimum-phase plants.
The cause is that the Kalman filter in the loop predicts with the command it sent, through its model of the actuators.


## 2. The LQG observer uses the commanded input

Plant, measurement and control:

$$
x_{k+1}=A\,x_k+B_{\text{true}}\,u_k+w_k,
\qquad
y_k=C\,x_k+v_k,
\qquad
u_k=-K\,\hat x_k .
$$

The observer propagates its own model of the input path:

$$
\hat x_{k+1}=A\,\hat x_k+\underbrace{B\,u_k}_{\text{commanded}}+L\,(y_k-C\,\hat x_k).
$$

**Perfect model, $B_{\text{true}}=B$.** For $e=x-\hat x$ the input cancels:

$$
e_{k+1}=(A-LC)\,e_k+w_k-L\,v_k,
\qquad
\begin{bmatrix}x\\ e\end{bmatrix}_{k+1}
=\begin{bmatrix}A-BK & BK\\ 0 & A-LC\end{bmatrix}
\begin{bmatrix}x\\ e\end{bmatrix}_k .
$$

The matrix is block-triangular, which is the separation principle: the closed-loop eigenvalues are those of $A-BK$ and $A-LC$, the error decays on its own, and the control loop sees it only as an outside, decaying disturbance.

**Actuator mismatch, $\Delta B=B_{\text{true}}-B$** (a gain error, saturation, friction, or a contact force that was planned but not realised). The control now enters the estimation error:

$$
e_{k+1}=(A-LC)\,e_k+\Delta B\,u_k
=(A-LC)\,e_k-\Delta B\,K\,(x_k-e_k),
$$

$$
\begin{bmatrix}x\\ e\end{bmatrix}_{k+1}
=\begin{bmatrix}A-B_{\text{true}}K & B_{\text{true}}K\\ -\Delta B\,K & A-LC+\Delta B\,K\end{bmatrix}
\begin{bmatrix}x\\ e\end{bmatrix}_k .
$$

The new $(2,1)$ block $-\Delta B\,K$ closes exactly the loop in the question:

$$
e\;\longrightarrow\;u=-K\hat x\;\longrightarrow\;\Delta B\,u\;\longrightarrow\;e .
$$

The observer predicts what the controller intended; the plant does something else, and the difference grows with the control itself.
The eigenvalues are no longer those of the two loops, and with aggressive $K$ or $L$ a small $\Delta B$ can make the closed loop unstable.

**Loop-transfer view.** Broken at the plant input, LQR's loop is $K(zI-A)^{-1}B$, which carries the guaranteed margins.
LQG's loop at the same point is

$$
K\,(zI-A+BK+LC)^{-1}L\,C\,(zI-A)^{-1}B ,
$$

which does not.
Loop transfer recovery (Doyle and Stein, *Robustness with Observers*, 1979) pushes the second toward the first by inflating the observer's fictitious noise at the plant input, at the cost of noise sensitivity.


## 3. The linear KF predicts with the measured acceleration, not the command

The [two-stage linear KF](wheeled_biped_state_estimation.md) takes attitude from the IMU and estimates $x=[p;\,v]$ with the IMU acceleration as the prediction input.
With $G=[\tfrac12\Delta t^2 I;\ \Delta t\,I]$ and the measured acceleration $\tilde a_k=a_k+n_k+b$ (true acceleration plus noise and bias):

$$
x_{k+1}=A\,x_k+G\,a_k,
\qquad
\hat x_{k+1}=A\,\hat x_k+G\,\tilde a_k+L\,(y_k-C\,\hat x_k),
$$

$$
\Rightarrow\qquad e_{k+1}=(A-LC)\,e_k-G\,(n_k+b)-L\,\nu_k .
$$

Neither $u$ nor $\Delta B$ appears: however wrong the actuator model, the torque saturation or the friction, the IMU measures the acceleration that actually happened.
The closed loop is a cascade,

$$
x_{k+1}=f\big(x_k,\ \pi(x_k-e_k)\big),
\qquad
e_{k+1}=(A-LC)\,e_k+d_k ,
$$

with $d_k$ driven by sensor noise only and no path from $x$ back to $e$.
If the MPC loop tolerates a bounded, decaying state error (input-to-state stability with respect to it), the cascade is stable.
This is the separation argument made exact for the estimator half; there is no formal guarantee for the nonlinear MPC and the filter together.

**Measured.** In the [ledge-drop runs](results/upkie_ledge_drop.md) the KF's height error has the same shape and decay (back within 5 mm in at most 0.17 s) whether it drives the MPC or is only logged while ground truth drives; nothing in the loop amplified it.


## 4. Where a coupling comes back: the measurement model

$d_k$ is sensor noise only while the measurement assumptions hold.
When the robot's motion breaks one, the error grows with the control error and the $(2,1)$ block reappears:

- **Slip in the tread or stance-foot velocity rows.** With $y_v=v-s(x,u)$,

  $$
  e_{k+1}=(A-LC)\,e_k+L\,s(x_k,u_k),
  $$

  and the coupling is $\partial s/\partial x$ and $\partial s/\partial u$: torque beyond traction, a wheel landing while spinning (the 0.6-0.7 m/s touchdown spikes of the Upkie jumps).
- **The contact schedule.** The command torque does not feed the estimator, but the MPC does write its planned contact flags into it, and they decide which rows are trusted.
  A wrong plan, for example a wheel over an edge while scheduled down, enters this way; that is the grey interval of the ledge-drop plots, harmless at 5 and 10 cm.
  Measured contacts (`contact.source: sensor`) remove this path.
- **Hardware only: the IMU's own attitude filter.** It takes sustained acceleration as gravity, so its roll and pitch drift during the long accelerations the control itself produces; the MPC then corrects a lean that is not there.
  MuJoCo's exact quaternion hides this.
- **Falling.** Past recovery every row is wrong, but by then the fall has already started; the estimate degrades as a consequence, as the G1 logs also show.


## 5. Is the IMU-driven prediction a discussed design choice?

The proprioceptive legged estimators all propagate with the IMU and correct with kinematics:
[Bloesch et al., RSS 2012](https://roboticsproceedings.org/rss08/p03.html);
[Rotella et al., 2014](https://alphaxiv.org/abs/1402.5450), which calls it a "sensor-based prediction model";
Hartley et al.'s contact-aided InEKF (RSS 2018);
the Cheetah 3 two-stage filter (Bledt et al., IROS 2018);
[Pronto (Camurri et al., 2020)](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2020.00068/epub).
In aided inertial navigation, using the accelerometers in place of a vehicle dynamics model is standard practice.
Their stated reason is model independence: no dynamics model, contact forces or torques are needed, and no assumptions about gait or terrain.
No legged paper found here frames the choice as avoiding the LQG problem; the decoupling of section 3 is a consequence of that choice, not its stated motivation.


## 6. Does an estimator built on a dynamics model fall back into it?

It depends on what drives the model's prediction.
Write the estimator as

$$
\hat x_{k+1}=\hat f(\hat x_k,\ \tau^{\text{in}}_k)+L_k\big(y_k-h(\hat x_k)\big),
\qquad
x_{k+1}=f(x_k,\ \tau_k)+w_k ,
$$

with $\hat f$ the estimator's dynamics model and $\tau^{\text{in}}$ the actuation fed into it.
Linearising $\hat f(x)-\hat f(\hat x)\approx F\,e$ gives

$$
e_{k+1}\approx(F-L H)\,e_k+\delta_k,
\qquad
\delta_k=\underbrace{f(x_k,\tau_k)-\hat f(x_k,\tau_k)}_{\text{model error}}
+\underbrace{\hat f(x_k,\tau_k)-\hat f(x_k,\tau^{\text{in}}_k)}_{\text{input error}} .
$$

| Prediction driven by | Input error | Model error | Coupling to the control |
| --- | --- | --- | --- |
| commanded torque, $\tau^{\text{in}}=u=\pi(\hat x)$ | $\hat B\,(\tau-u)$: the actuator mismatch, the $\Delta B\,u$ of LQG | present | **the LQG $(2,1)$ block**, plus a common-mode error (below) |
| measured torque or force, $\tau^{\text{in}}=\tau+n_\tau$ | $-\hat B\,n_\tau$: sensor noise | present, and it scales with the actual forces | reduced: proportional to the dynamic-parameter error, not the actuator error |
| measured acceleration (IMU-driven kinematic filter) | none | none in the prediction; only in the measurement model | none, until a measurement assumption breaks (section 4) |

**Measured forces still carry a smaller coupling.** With a mass error $\hat m=m(1+\varepsilon)$ in a centroidal model,

$$
\delta_{v,k}=\Delta t\left(\frac{F_k}{m}-\frac{F_k}{\hat m}\right)\approx\Delta t\,\varepsilon\,\frac{F_k}{m},
$$

which grows with the contact force $F_k$ that the controller sets, so the $(2,1)$ block is present with size about $\varepsilon\,\partial F/\partial x$: the relative parameter error rather than the actuator error.

**Common-mode error.** When the estimator and the controller use the same model $\hat f$, their errors are correlated: the estimator's prediction says the plan is working exactly as the controller expects, and only the measurement update can disagree.
With a confident model (small process noise) the estimate leans toward the controller's belief.
The published remedy is to let the estimator absorb the mismatch in extra disturbance states (a CoM offset, an external force), as offset-free MPC does in process control.


## 7. Papers that put dynamics into the estimator

| Paper | Robot | Dynamics in the estimator | Actuation input | Estimates | Model-error handling | Coupling risk (this note's analysis) |
| --- | --- | --- | --- | --- | --- | --- |
| [Stephens, ICRA 2011](https://dblp.uni-trier.de/rec/conf/icra/Stephens11.html), *State estimation for force-controlled humanoid balance using simple models in the presence of modeling error* | Sarcos force-controlled humanoid | linear inverted pendulum | measured contact forces (not verified here) | CoM state | step-disturbance states (CoM offset or external force); a naive estimator ignoring the error is inaccurate | low with the disturbance states |
| [Xinjilefu, Feng, Atkeson, ICRA 2014](https://doi.org/10.1109/ICRA.2014.6906609), *Decoupled state estimation for humanoids using full-body dynamics* | Atlas | full-body dynamics, split into independent sub-filters (steady-state KF) | Atlas sensing (not verified here) | full-body state | decoupling | depends on the actuation input: low if sensed, LQG-like if commanded |
| [Xinjilefu, Feng, Atkeson, IROS 2014](https://doi.org/10.1109/IROS.2014.6942679), *Dynamic state estimation using quadratic programming* | Atlas | full-body dynamics in a QP | Atlas sensing (not verified here) | full-body state | the modelling error is a decision variable | lower: the QP can explain mismatch as model error |
| [Rotella, Herzog, Schaal, Righetti, Humanoids 2015](https://arxiv.org/pdf/1507.04401), *Humanoid momentum estimation using sensed contact wrenches* | Sarcos humanoid | centroidal momentum dynamics | measured contact wrenches (force/torque sensors) | CoM, momentum, external wrench, CoM offset | external-wrench and offset states; base from the kinematic-inertial filter | low |
| [Piperakis, Koskinopoulou, Trahanias, RA-L 2018](https://researchportal.hw.ac.uk/en/publications/nonlinear-state-estimation-for-humanoid-robot-walking/), *Nonlinear state estimation for humanoid robot walking* ([SEROW](https://github.com/mrsp/serow)) | NAO | Newton-Euler CoM dynamics with angular-momentum rate | measured ground reaction forces | CoM position and velocity, external forces | external-force states; cascade behind an IMU-kinematic base estimator | low |
| [Fourmy, Flayols, Léziart, Mansard, Solà, ICRA 2021](https://www.iri.upc.edu/download/scidoc/2308), *Contact forces preintegration for estimation in legged robotics using factor graphs* | Solo12 | centroidal dynamics, contact forces preintegrated like IMU data | contact forces as measurements | base and centroidal states, tightly coupled | factor-graph uncertainty | low to moderate: force accuracy matters |
| [Bledt, Wensing, Ingersoll, Kim, ICRA 2018](https://dspace.mit.edu/handle/1721.1/120350), *Contact model fusion for event-based locomotion in unstructured terrains* | Cheetah 3 | discrete-time generalised-momentum observer | joint torques | contact events, fused with gait phase and kinematics in a KF (99.3 % accuracy, 4-5 ms delay) | fusion with other contact priors | affects contact decisions only, not the pose |
| [Pannocchia and Rawlings, AIChE J. 2003](https://zendy.io/title/10.1002/aic.690490213), *Disturbance models for offset-free model-predictive control* | process control | the plant model | the commanded input | states and integrating disturbances | disturbance states absorb plant-model mismatch | the LQG structure by design; robustness depends on the disturbance model and observer |
| [Flayols et al., Humanoids 2017](https://hal-univ-tlse3.archives-ouvertes.fr/hal-01574819), *Experimental evaluation of simple estimators for humanoid robots* | HRP-2 | none: simple estimators on off-the-shelf orientation estimates | none | base pose and velocity | - | the comparison point: simple estimators match more complex ones in accuracy |

**Good or bad choice?**

- **Bad: dynamics driven by the commanded torque, estimating the base state, sharing the controller's model, without disturbance states.**
  It has the LQG coupling of section 2 and the common-mode error of section 6, and none of the legged estimators above does this for the base pose.
- **Good: dynamics with measured forces, plus disturbance or offset states, for what kinematics and the IMU cannot give.**
  That means CoM offset, momentum, external wrench and contact events, cascaded behind or fused with an IMU-kinematic base estimate.
  Every published design above that uses dynamics is in this category: the dynamics adds information the kinematic-inertial filter lacks, and the measured forces and disturbance states keep the actuation error out.
- **For this repository:** the base estimate needs no dynamics, and the WB-ID MPC already predicts with the full model and commanded torques.
  An estimator propagating the same model with the same commands would inherit the MPC's own model errors, such as the approximate joint friction (tanh against MuJoCo's friction-loss constraint).
  Candidate uses of dynamics are an external-force estimate for pushes, CoM offset, and contact detection from motor torques, each kept out of the base pose.


## References

- J. C. Doyle, *Guaranteed Margins for LQG Regulators*, IEEE TAC 23(4), 1978. [PDF](https://doyle.caltech.edu/images/doyle/6/62/GuaranteedMarginsLQGRegs.pdf), [summary](https://labs.utdallas.edu/conlab/guaranteed-margins-for-lqg-regulators).
- J. C. Doyle and G. Stein, *Robustness with Observers*, IEEE TAC 24(4), 1979.
- M. Bloesch et al., *State Estimation for Legged Robots: Consistent Fusion of Leg Kinematics and IMU*, RSS 2012. [Proceedings](https://roboticsproceedings.org/rss08/p03.html).
- N. Rotella et al., *State Estimation for a Humanoid Robot*, IROS 2014. [arXiv](https://alphaxiv.org/abs/1402.5450).
- M. Camurri et al., *Pronto: A Multi-Sensor State Estimator for Legged Robots*, Frontiers 2020. [Article](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2020.00068/epub).
- The dynamics-based estimators: see the table in section 7.
