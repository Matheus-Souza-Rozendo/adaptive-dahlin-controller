[![MATLAB](https://img.shields.io/badge/MATLAB-0076A8?style=for-the-badge&logo=mathworks&logoColor=white)](https://www.mathworks.com/products/matlab.html)  
[![GNU Octave](https://img.shields.io/badge/GNU%20Octave-0790C0?style=for-the-badge&logo=octave&logoColor=white)](https://www.gnu.org/software/octave/)  
[![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)](https://www.arduino.cc/)  
[![C](https://img.shields.io/badge/C-A8B9CC?style=for-the-badge&logo=c&logoColor=white)](https://en.wikipedia.org/wiki/C_(programming_language))  

# Adaptive Dahlin Controller (Self-Tuning Regulator)

The main goal is to compare the performance of a fixed-parameter controller versus a controller that uses **Recursive Least Squares (RLS)** estimation to adapt online to the plant parameters.

## Main Concepts and Demonstrated Skills

This project is a complete end-to-end practical demonstration of a control system — from theoretical modeling to hardware implementation.

* **System Modeling:** Derivation of the transfer function for a first-order RC circuit.  
* **Digital Control:** Discretization of a continuous-time system G(s) into a discrete-time model  G(z) using the step-invariance method.  
* **Controller Design:**
  * **Traditional Dahlin:** Calculation of a discrete-time controller to meet first-order closed-loop response specifications $K'=1$,
$\tau'=\tau/2\$.
  * **Adaptive Dahlin (STR):** Implementation of a self-tuning controller.  
* **Adaptive Systems:** Use of the **Recursive Least Squares (RLS)** method with a forgetting factor $\lambda$ to estimate the plant parameters 
  $a_1, b_1$
in real time.  
* **Simulation:** MATLAB/Octave used to simulate and validate the closed-loop controller behavior.  
* **Practical Implementation (Hardware-in-the-loop):** Experimental validation of the designed controllers using an Arduino Mega and Octave platform.  

## Methodology

The project was divided into three main stages:

### 1. Plant Modeling

The plant is an RC circuit. From nodal analysis, the following continuous-time transfer function was derived:

$$
G(s) = \frac{Y(s)}{U(s)} = \frac{\frac{R_1}{R_1 + R_2}}{C \cdot \frac{R_1 R_2}{R_1 + R_2} \cdot s + 1} = \frac{K}{\tau s + 1}
$$

Using nominal values ($R_1 = 9.96k\Omega$, $R_2 = 9.79k\Omega$, $C = 448\mu F\$), we obtain:

* **Gain:** K ≈ 0.5043  
* **Time constant:** $\tau$ ≈ 2.2118 s  

With a sampling period $T = 0.1$ s, the discrete-time plant 

$$
G(z) = \frac{b_1 z^{-1}}{1 + a_1 z^{-1}} 
$$

has parameters:

$a_1 = -e^{-T/\tau} \approx -0.9558 $ 

$b_1 = K(1 + a_1) \approx 0.0223$

### 2. Traditional Dahlin Controller

Designed for a closed-loop response with K' = 1 and $\tau' = \tau/2 \approx 1.1059$ s.  

The resulting difference equation of the controller is:

$$
u(n) = u(n-1) + 3.8789 \cdot e(n) - 3.7075 \cdot e(n-1)
$$

### 3. Adaptive Dahlin Controller (STR)

![Controller Architecture](/images/controller_architecture.png)

*(Caption: Controller Structure)*

This controller implements the same logic, but the parameters $a_1$ and $b_1$ are not fixed.  

They are estimated at each time step n by the RLS estimator, producing $\hat{a}_1(n)$ and $\hat{b}_1(n)$.

These estimates are used to recalculate the controller gains in real time:

$$
u(n) = u(n-1) + K_c(n) \cdot e(n) - K_d(n) \cdot e(n-1)
$$

Where $K_c(n)$ and $K_d(n)$ are functions of $\hat{a}_1(n)$ and $\hat{b}_1(n)$.

## Results

Both controllers were simulated and tested on the real plant.

### Parameter Evolution (Adaptive)

The plot below shows the convergence of the estimated parameters $\hat{a}_1$ and $\hat{b}_1$ to the plant’s true values $a_1 \approx -0.9558$, 
$b_1 \approx 0.0223$
during simulation.  
The fast convergence validates the effectiveness of the RLS estimator.

![Parameter Evolution](/images/parametersRLS.png)

*(Caption: Evolution of estimated parameters)*

### Practical Comparison (Hardware)

The most significant result comes from the practical Arduino experiment.  
The plot compares the plant’s open-loop response with the Traditional Dahlin and the Adaptive Dahlin controllers.

![Experiment](/images/test_response.png)

*(Caption: Real system responses – open loop vs. traditional Dahlin vs. adaptive Dahlin)*

Both controllers successfully track the reference, unlike the open-loop system.  
The Adaptive Dahlin controller exhibits a noticeably faster and more aggressive response, with a shorter settling time during step transitions.  
The Traditional Dahlin controller is smoother but shows a larger overshoot in the practical test — something the adaptive controller managed to compensate for.

## Repository Structure

* **Controller_Dahlin.m** – Simulation script for the **Traditional Dahlin Controller**  
* **adaptative_Controller_Dahlin.m** – Simulation script for the **Adaptive Dahlin Controller** with RLS  
* `/pratic_test` – Contains scripts and `.mat` data files for the practical Arduino implementation  

## 🚀 How to Run

### Simulation

1. Clone this repository.  
2. Open MATLAB or Octave.  
3. Run `/Controller_Dahlin.m` to simulate the traditional controller.  
4. Run `/adaptative_Controller_Dahlin.m` to simulate the adaptive controller.  

### Practical Experiment

* The practical experiment requires Arduino Mega hardware.  
* The scripts in the `/pratic_test` folder are used to compute the control signal and send it to the Arduino via Serial communication.  
