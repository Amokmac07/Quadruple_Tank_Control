
<body>

<h1>Quadruple Tank Problem</h1>

<p>This project involves using estimation techniques to control the setup of a quadruple tank process to a desired set point.</p>

<h2>System Description</h2>

<p>The quadruple tank process is described in the paper titled "The quadruple-tank process: a multivariable laboratory process with an adjustable zero" by K. H. Johansson [1]. The system consists of four interconnected tanks as shown below:</p>
<!-- Include the image for tank.jpg -->
<img src="Pics/tank.jpg" alt="Quadruple Tank Process" style="width: 450px;"><sub>[1]</sub>
<p>Using first principle method we describe the system as per the equations</p>
<img src="Pics/tank_equations.jpeg" alt="Temperature Parameters" style="width: 400px;"><sub>[1]</sub>
<p>The additional data given in the paper [1] is shown below</p>
<img src="Pics/data.jpeg" alt="Temperature Parameters" style="width: 350px;"><sub>[1]</sub>
<h2>State Space Representation</h2>

<p>The state space representation of the system is provided in the paper [1]. The parameters A, B, and C are described in the images below:</p>

<!-- Display images side by side -->
<div style="display: flex; flex-wrap: wrap;">
  <!-- Include the image for A.jpg -->
  <div style="width: 45%;">
    <img src="Pics/A.jpg" alt="Parameter A" style="width: 300px">
    <p style="text-align: center;"><sub>[1]</sub></p>
  </div>
  <!-- Include the image for B.jpg -->
  <div style="width: 45%;">
    <img src="Pics/B.jpg" alt="Parameter B" style="width: 300px;">
    <p style="text-align: center;"> <sub>[1]</sub></p>
  </div>
</div>


<!-- Include the image for C.jpg -->
<img src="Pics/C.jpg" alt="Parameter C" style="width: 200px;"><sub>[1]</sub>

<p>Additionally, the temperature parameters Ti are shown in the image below:</p>

<!-- Include the image for Ti.jpg -->
<img src="Pics/Ti.jpg" alt="Temperature Parameters" style="width: 300px;"><sub>[1]</sub>

<h2>Measurements Generation</h2>
 <p><em>(refer to  file Data_Generation)</em></p>
    <p>This code simulates a system of four interconnected tanks (as shown in the image above) and generates data representing the measurements available from the sensors placed in each tank. Here's a brief description of the system and data generation process:</p>
    <ul>
        <li><h3>System Description</h3><ul>
                <li>The system consists of four tanks interconnected with pipes.</li>
                <li>Each tank has an input flow rate and is subjected to gravitational forces.</li>
                <li>The height of the liquid in each tank varies over time due to inflow and outflow dynamics as shown earlier.</li>
            </ul>
        </li>
        <li><h3>Data Generation</h3>
            <ul>
                <li>The differential equations governing the dynamics of the system are defined based on the principles of fluid dynamics and mass balance.</li>
                <li>The system of differential equations is numerically solved using the <code>odeint</code> function from the <code>scipy.integrate</code> module.</li>
                <li>The simulation is conducted over a specified time range (0 to 10000 seconds) with 10000 time points.</li>
                <li>The resulting time series data includes the heights of liquid in each tank (<code>Height_1</code> to <code>Height_4</code>) at each time point.</li>
                <li>This data represents the measurements available from the sensors placed in each tank, providing insights into the dynamics of the system over time.</li>
            </ul>
        </li>
        <li><h3>Data Export</h3>
            <ul>
                <li>The generated data is stored in a pandas DataFrame.</li>
                <li>The DataFrame is exported to an Excel file named "tank_data.xlsx" for further analysis or visualization.</li>
            </ul>
        </li>
</ul>
<h2>Methods</h2>

<h3>Part 1: Kalman Filter</h3>

<p>In this part, the Kalman Filter was applied to estimate states and analyze the residuals.The following algorithm was executed : - </p>
<img src="https://ece.montana.edu/seniordesign/archive/SP14/UnderwaterNavigation/website_images/kalman_f.PNG" alt="Parameter C" style="width: 450px;"><sub>[2]</sub>
<p>Now on simulating the filter we observe that the readings converge to a final point . First looking at variation of x_prior followed by variation of x_posterior </p>
<img src="Pics/pic_1.jpg" alt="final state " style="width: 1000px">
<img src="Pics/pic_2.jpg" alt="final state " style="width: 1000px">
<p>The covariance matrix P thus reduces in value thus showing higher accuracy in estimation , the Kalman gain as well reduces to zero.</p>
<img src="Pics/pic_3.jpg" alt="final state " style="width: 400px">
<img src="Pics/pic_4.jpg" alt="final state " style="width: 400px">
<p>Thus we check the accuracy of our predictions by checking the residuals and innovations which tend to zero as we see below</p>
<img src="Pics/pic_5.jpg" alt="final state " style="width: 1000px">
<img src="Pics/pic_6.jpg" alt="final state " style="width: 1000px">


<p>
        The simulation of the Kalman Filter demonstrates a successful estimation of states with convergence towards accurate values. Key observations from the simulation results include:
    </p>
    <ul>
        <li>
            <strong>State Estimation Convergence:</strong>
            The graphs of <i>x<sub>prior</sub></i> and <i>x<sub>posterior</sub></i> illustrate that the state estimates converge to a final point. This indicates that the filter effectively tracks the true state over time.
        </li>
        <li>
            <strong>Covariance Matrix Reduction:</strong>
            The reduction in the values of the covariance matrix <i>P</i> signifies an increase in estimation accuracy. As the filter processes more measurements, it becomes more confident in its state estimates, leading to a decrease in uncertainty.
        </li>
        <li>
            <strong>Kalman Gain Reduction:</strong>
            The Kalman gain reduces to zero, which is expected as the filter converges. A lower Kalman gain implies that new measurements have less impact on the state estimates, indicating a high level of confidence in the current state estimate.
        </li>
        <li>
            <strong>Residuals and Innovations:</strong>
            The residuals and innovations approach zero, confirming the accuracy of the predictions. This behavior suggests that the filter's estimates align closely with the actual measurements, validating the effectiveness of the Kalman Filter in minimizing prediction errors.
        </li>
    </ul>
    <p>
        Overall, the simulation results confirm the Kalman Filter's capability to accurately estimate states by reducing uncertainty and refining predictions over time. The convergence of state estimates, reduction in covariance values, diminishing Kalman gain, and minimal residuals collectively demonstrate the filter's proficiency in state estimation tasks.
    </p>



<h3>Part 2: Particle Filter</h3>

<p>In this part, we will apply the Particle Filter to obtain visible convergence. Due to limited computational power, running the Particle Filter may be difficult.</p>

<h3>Part 3: Extended Kalman Filter with Model Predictive Control</h3>

<p>In this part, we will apply the Extended Kalman Filter to estimate the state and use Model Predictive Control to take the system to the desired set point.</p>



<h2>References</h2>

<ol>
  <li>K. H. Johansson, "The quadruple-tank process: a multivariable laboratory process with an adjustable zero," in IEEE Transactions on Control Systems Technology, vol. 8, no. 3, pp. 456-465, May 2000, <a href="https://doi.org/10.1109/87.845876">doi: 10.1109/87.845876</a>.</li>
  <li>“Kalman filter.” https://ece.montana.edu/seniordesign/archive/SP14/UnderwaterNavigation/kalman_filter.html</li>
</ol>

</body>


