let globalChargeStatus = 0;
const control = document.getElementById("control-container");

// Function to initialize WebSocket connection
function connectWebSocket() {
  // Update the UI
  const chargeElement = document.getElementById("battery-charge");
  const voltageElement = document.getElementById("battery-voltage");
  const icCharger = document.getElementById("ic-charger");
  const icBattery = document.getElementById("ic-battery");
  const xAxisElement = document.getElementById("x-axis"); // New Element
  const yAxisElement = document.getElementById("y-axis"); // New Element

  ws = new WebSocket("ws://" + window.location.hostname + "/ws");

  // Set binary type to ArrayBuffer
  ws.binaryType = "arraybuffer";

  ws.onopen = function () {
    console.log("WebSocket connection established");
  };

  ws.onmessage = function (event) {
    // Ensure the data is received as an ArrayBuffer
    if (!(event.data instanceof ArrayBuffer)) {
      console.error("Received data is not an ArrayBuffer.");
      return;
    }

    const buffer = event.data;
    const dataView = new DataView(buffer);

    // Verify the packet length
    if (dataView.byteLength !== 14) {
      console.error(`Unexpected packet size: ${dataView.byteLength} bytes. Expected 14 bytes.`);
      return;
    }

    // Parse batteryVoltage (float32) from bytes 0-3
    const batteryVoltage = dataView.getFloat32(0, true); // true for little-endian

    // Parse batterySoC (uint8) from byte 4
    const batterySoC = dataView.getUint8(4);

    // Parse chargerStatus (uint8) from byte 5
    const batteryStatusByte = dataView.getUint8(5);
    const batteryChargeStatus = batteryStatusByte;
    let batteryStatusString;

    switch (batteryChargeStatus) {
      case 0xa0:
        // batteryStatusString = "Running on battery";
        globalChargeStatus = 0;
        icBattery.style.display = "block";
        icCharger.style.display = "none";
        break;
      case 0xa1:
        // batteryStatusString = "Battery charging";
        globalChargeStatus = 1;
        icBattery.style.display = "none";
        icCharger.style.display = "block";
        break;
      case 0xa2:
        // batteryStatusString = "Battery charged";
        globalChargeStatus = 1;
        icBattery.style.display = "none";
        icCharger.style.display = "block";
        break;
      default:
        // batteryStatusString = "Unknown";
        globalChargeStatus = 1;
        icBattery.style.display = "none";
        icCharger.style.display = "block";
    }

    // **New Additions Start Here**

    // Parse gx (float32) from bytes 6-9
    const gx = dataView.getFloat32(6, true); // true for little-endian

    // Parse gy (float32) from bytes 10-13
    const gy = dataView.getFloat32(10, true); // true for little-endian

    // **New Additions End Here**

    if (chargeElement && batterySoC !== undefined) {
      chargeElement.textContent = `${batterySoC}%`;
    } else {
      chargeElement.textContent = undefined;
    }

    if (voltageElement && batteryVoltage !== undefined) {
      voltageElement.textContent = `${batteryVoltage.toFixed(2)}V`;
    } else {
      voltageElement.textContent = `0.00V`;
    }

    // **New Additions Start Here**

    if (xAxisElement && gx !== undefined) {
      xAxisElement.textContent = `${gx.toFixed(2)}`;
    } else {
      xAxisElement.textContent = `0.00`;
    }

    if (yAxisElement && gy !== undefined) {
      yAxisElement.textContent = `${gy.toFixed(2)}`;
    } else {
      yAxisElement.textContent = `0.00`;
    }

    // **New Additions End Here**
  };

  ws.onclose = function () {
    console.log("WebSocket connection closed, retrying in 1 second...");
    setTimeout(connectWebSocket, 1000);
  };

  ws.onerror = function (error) {
    console.error("WebSocket Error: ", error);
  };
}

// Establish WebSocket connection
connectWebSocket();

function sendMotorData(leftMotor, rightMotor) {
  // Create a buffer of 4 bytes (2 bytes for each motor value as Int16)
  const buffer = new ArrayBuffer(4);
  const dataView = new DataView(buffer);

  // Store the left motor value (Int16 at byte offset 0)
  dataView.setInt16(0, leftMotor, true); // true for little-endian

  // Store the right motor value (Int16 at byte offset 2)
  dataView.setInt16(2, rightMotor, true); // true for little-endian

  // Send the buffer via WebSocket if the connection is open
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(buffer);
    console.log("Sent motor data via WebSocket:", leftMotor, rightMotor);
  }
}

let limit = 120;
const driftCompensation = 20;

let animationFrameId, startX, startY;
const motionIndicator = document.getElementById("motionIndicator");
const precisionIndicator = document.getElementById("precisionIndicator");

const startIndicator = document.getElementById("startIndicator");
const verticalLine = document.getElementById("verticalLine");
const horizontalLine = document.getElementById("horizontalLine");
const driftIndicator = document.getElementById("driftIndicator");
const verticalLimitIndicator = document.getElementById("verticalLimitIndicator");
const horizontalLimitIndicator = document.getElementById("horizontalLimitIndicator");

function initializeTracking({ clientX, clientY, changedTouches }) {
  if (clientX && clientY) {
    startX = clientX;
    startY = clientY;
  } else if (changedTouches) {
    const firstTouch = changedTouches[0];
    startX = firstTouch.clientX;
    startY = firstTouch.clientY;
  }

  // Position the motionIndicator at the starting point
  // motionIndicator.style.left = `${startX}px`;
  // motionIndicator.style.top = `${startY}px`;

  const rect = control.getBoundingClientRect(); // Get wrapper's position
  const relativeX = startX - rect.left; // X relative to wrapper
  const relativeY = startY - rect.top; // Y relative to wrapper

  motionIndicator.style.left = `${relativeX}px`;
  motionIndicator.style.top = `${relativeY}px`;

  // Show the motionIndicator
  motionIndicator.style.display = "flex";

  startIndicator.style.display = "block";
  startIndicator.style.left = `${relativeX}px`;
  startIndicator.style.top = `${relativeY}px`;

  horizontalLine.style.display = "block";
  horizontalLine.style.top = `${relativeY}px`;

  verticalLine.style.display = "block";
  verticalLine.style.left = `${relativeX}px`;

  verticalLimitIndicator.style.display = "block";
  verticalLimitIndicator.style.left = `${relativeX}px`;
  verticalLimitIndicator.style.width = `${limit * 2 + 1}px`;

  horizontalLimitIndicator.style.display = "block";
  horizontalLimitIndicator.style.top = `${relativeY}px`;
  horizontalLimitIndicator.style.height = `${limit * 2 + 1}px`;

  if (globalChargeStatus !== 0) {
    motionIndicator.classList.add("disabled-border");
    precisionIndicator.classList.add("disabled-fill");
  } else {
    motionIndicator.classList.remove("disabled-border");
    precisionIndicator.classList.remove("disabled-fill");
  }

  if (driftCompensation !== 0) {
    driftIndicator.style.display = "block";
    driftIndicator.style.left = `${relativeX}px`;
    driftIndicator.style.width = `${driftCompensation * 2}px`;
  }

  control.addEventListener("mousemove", handleMovement);
  control.addEventListener("touchmove", handleMovement);
}

function resetTracking() {
  startX = undefined;
  startY = undefined;
  // Hide the motionIndicator
  motionIndicator.style.display = "none";
  startIndicator.style.display = "none";
  verticalLine.style.display = "none";
  horizontalLine.style.display = "none";
  verticalLimitIndicator.style.display = "none";
  horizontalLimitIndicator.style.display = "none";
  driftIndicator.style.display = "none";

  control.removeEventListener("mousemove", handleMovement);
  control.removeEventListener("touchmove", handleMovement);

  const motorA = document.getElementById("motorA"); // New Element
  const motorB = document.getElementById("motorB"); // New Element
  motorA.textContent = `0`;
  motorB.textContent = `0`;

  // Call sendMotorData with the left and right motor values
  sendMotorData(parseInt(0), parseInt(0));
}

function mapValue(value, driftCompensation, limit) {
  return parseInt((value / (limit - driftCompensation)) * limit);
}

function handleMovement({ clientX, clientY, changedTouches }) {
  // Cancel the previous animation frame if it's pending
  if (animationFrameId) {
    cancelAnimationFrame(animationFrameId);
  }

  animationFrameId = requestAnimationFrame(() => {
    let xCoord, yCoord, xMovement, yMovement;

    if (clientX && clientY) {
      xCoord = clientX;
      yCoord = clientY;
    } else if (changedTouches) {
      const firstTouch = changedTouches[0];
      xCoord = firstTouch.clientX;
      yCoord = firstTouch.clientY;
    }

    // Calculate raw movements
    xMovement = xCoord - startX;
    yMovement = startY - yCoord;

    if (-driftCompensation < xMovement && xMovement < driftCompensation) {
      xMovement = 0;
    }

    // Constrain xMovement within -160 to 160
    if (xMovement > limit) {
      xMovement = limit;
    } else if (xMovement < -limit) {
      xMovement = -limit;
    }

    // Constrain yMovement within -160 to 160
    if (yMovement > limit) {
      yMovement = limit;
    } else if (yMovement < -limit) {
      yMovement = -limit;
    }

    // Calculate new position
    const newX = startX + xMovement;
    const newY = startY - yMovement; // Subtract because yMovement was startY - clientY

    // Update motionIndicator's position
    // motionIndicator.style.left = `${newX}px`;
    // motionIndicator.style.top = `${newY}px`;

    const rect = control.getBoundingClientRect(); // Get wrapper's position
    const relativeX = newX - rect.left; // X relative to wrapper
    const relativeY = newY - rect.top; // Y relative to wrapper

    motionIndicator.style.left = `${relativeX}px`;
    motionIndicator.style.top = `${relativeY}px`;

    if (xMovement < 0) {
      xMovement = xMovement + driftCompensation;
    } else if (xMovement > 0) {
      xMovement = xMovement - driftCompensation;
    }

    console.log("Coordinates", mapValue(xMovement, driftCompensation, limit), yMovement);

    // Map joystick movements to motor speeds
    // let leftMotor = yMovement + xMovement;
    // let rightMotor = yMovement - xMovement;

    // let leftMotor = yMovement + xMovement;
    // let rightMotor = yMovement - xMovement;
    // console.log("leftMotor", leftMotor, "rightMotor", rightMotor);

    const motorPWMs = calculateMotorPWM(mapValue(xMovement, driftCompensation, limit), yMovement, limit);
    console.log("Left Motor PWM", parseInt((motorPWMs.leftMotor * 255) / limit), "Right Motor PWM", parseInt((motorPWMs.rightMotor * 255) / limit));

    let leftMotor = parseInt((motorPWMs.leftMotor * 255) / limit); //motorPWMs.leftMotor;
    let rightMotor = parseInt((motorPWMs.rightMotor * 255) / limit); //motorPWMs.rightMotor;

    const motorA = document.getElementById("motorA"); // New Element
    const motorB = document.getElementById("motorB"); // New Element
    motorA.textContent = `${rightMotor}`;
    motorB.textContent = `${leftMotor}`;

    // Optional: Constrain motor values to PWM range (-255 to 255)
    // leftMotor = Math.max(-255, Math.min(255, leftMotor)).toFixed(0);
    // rightMotor = Math.max(-255, Math.min(255, rightMotor)).toFixed(0);

    if (isNaN(leftMotor)) leftMotor = 0;
    if (isNaN(rightMotor)) rightMotor = 0;

    // console.log("motorPWM", leftMotor, rightMotor);

    // Call sendMotorData with the left and right motor values
    sendMotorData(parseInt(rightMotor), parseInt(leftMotor));

    if (globalChargeStatus !== 0) {
      motionIndicator.classList.add("disabled-border");
      precisionIndicator.classList.add("disabled-fill");
    } else {
      motionIndicator.classList.remove("disabled-border");
      precisionIndicator.classList.remove("disabled-fill");
    }
  });
}

function calculateMotorPWM(x, y, limit) {
  // Define maximum PWM value based on the absolute value of Y
  const maxPWM = Math.max(0, Math.min(limit, Math.abs(y))); // Use absolute Y for backward movement

  // Clamp X values to the range of -limit to limit
  x = Math.max(-limit, Math.min(limit, x));

  let leftMotorPWM = 0; // Initialize left motor PWM
  let rightMotorPWM = 0; // Initialize right motor PWM

  // Determine motor speeds based on Y and X coordinates
  if (y > 0) {
    // Top half (forward)
    if (x < 0) {
      // Moving left
      leftMotorPWM = maxPWM; // Left motor gets full power
      rightMotorPWM = maxPWM + (x / limit) * maxPWM; // Right motor decreases in power
    } else if (x > 0) {
      // Moving right
      rightMotorPWM = maxPWM; // Right motor gets full power
      leftMotorPWM = maxPWM - (x / limit) * maxPWM; // Left motor decreases in power
    } else {
      // Centered (x = 0)
      leftMotorPWM = maxPWM; // Both motors get the full power from Y
      rightMotorPWM = maxPWM;
    }
  } else {
    // Bottom half (backward)
    if (x < 0) {
      // Moving left
      leftMotorPWM = -maxPWM; // Left motor goes full reverse
      rightMotorPWM = -maxPWM + (Math.abs(x) / limit) * maxPWM; // Right motor decreases in reverse
    } else if (x > 0) {
      // Moving right
      rightMotorPWM = -maxPWM; // Right motor goes full reverse
      leftMotorPWM = -maxPWM + (Math.abs(x) / limit) * maxPWM; // Left motor decreases in reverse
    } else {
      // Centered (x = 0)
      leftMotorPWM = -maxPWM; // Both motors go full reverse
      rightMotorPWM = -maxPWM;
    }
  }

  // Ensure PWM values are clamped between -limit and limit
  leftMotorPWM = Math.max(-limit, Math.min(limit, leftMotorPWM));
  rightMotorPWM = Math.max(-limit, Math.min(limit, rightMotorPWM));

  return {
    leftMotor: Math.round(leftMotorPWM),
    rightMotor: Math.round(rightMotorPWM),
  };
}

function addListeners() {
  control.addEventListener("mousedown", initializeTracking);
  control.addEventListener("mouseup", resetTracking);
  control.addEventListener("touchstart", initializeTracking);
  control.addEventListener("touchend", resetTracking);
}

addListeners();
