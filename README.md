<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Quadruple Tank Problem</title>
<style>
  .image-container {
    display: flex;
    justify-content: space-between;
  }
  .image-container img {
    width: 45%; /* Adjust width as needed */
  }
</style>
</head>
<body>

<h1>Quadruple Tank Problem</h1>

<p>This project involves using estimation techniques to control the setup of a quadruple tank process to a desired set point.</p>

<h2>System Description</h2>

<p>The quadruple tank process is described in the paper titled "The quadruple-tank process: a multivariable laboratory process with an adjustable zero" by K. H. Johansson [1]. The system consists of four interconnected tanks as shown below:</p>
<!-- Include the image for tank.jpg -->
<img src="Pics/tank.jpg" alt="Quadruple Tank Process" style="width: 450px;"><sub>[1]</sub>

<h2>State Space Representation</h2>

<p>The state space representation of the system is provided in the paper [1]. The parameters A, B, and C are described in the images below:</p>

<div class="image-container">
  <!-- Include the image for A.jpg -->
  <img src="Pics/A.jpg" alt="Parameter A"><sub>[1]</sub>
  <!-- Include the image for B.jpg -->
  <img src="Pics/B.jpg" alt="Parameter B"><sub>[1]</sub>
</div>

<!-- Include the image for C.jpg -->
<img src="Pics/C.jpg" alt="Parameter C" style="width: 200px;"><sub>[1]</sub>

<p>Additionally, the temperature parameters Ti are shown in the image below:</p>

<!-- Include the image for Ti.jpg -->
<img src="Pics/Ti.jpg" alt="Temperature Parameters" style="width: 300px;"><sub>[1]</sub>

<h2>Methods</h2>

<h3>Part 1: Kalman Filter</h3>

<p>In this part, we will apply the Kalman Filter to estimate states and analyze the residuals.</p>

<h3>Part 2: Particle Filter</h3>

<p>In this part, we will apply the Particle Filter to obtain visible convergence. Due to limited computational power, running the Particle Filter may be difficult.</p>

<h3>Part 3: Extended Kalman Filter with Model Predictive Control</h3>

<p>In this part, we will apply the Extended Kalman Filter to estimate the state and use Model Predictive Control to take the system to the desired set point.</p>



<h2>References</h2>

<ol>
  <li>K. H. Johansson, "The quadruple-tank process: a multivariable laboratory process with an adjustable zero," in IEEE Transactions on Control Systems Technology, vol. 8, no. 3, pp. 456-465, May 2000, <a href="https://doi.org/10.1109/87.845876">doi: 10.1109/87.845876</a>.</li>
</ol>

</body>


