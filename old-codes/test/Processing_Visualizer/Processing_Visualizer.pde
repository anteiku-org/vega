import processing.serial.*;

Serial myPort;
String data="";
float roll, pitch, yaw;

void setup() {
  size(800, 600, P3D);
  
  // IMPORTANT: Set your COM port here!
  // Look at your Arduino IDE to see which COM port your ESP32 is on (e.g. "COM3", "COM5")
  String portName = "COM14"; // <--- CHANGE THIS
  
  try {
    myPort = new Serial(this, portName, 115200);
    myPort.bufferUntil('\n');
    println("Connected to " + portName);
  } catch (Exception e) {
    println("Could not connect to " + portName + ". Make sure the Arduino IDE Serial Monitor is CLOSED!");
    
    // Uncomment the next line to see a list of available ports if you're stuck:
    // printArray(Serial.list());
  }
}

void draw() {
  background(230);
  
  // Add some lighting to make it look 3D
  lights();
  directionalLight(255, 255, 255, -1, 1, -1);
  
  // Move to the center of the screen
  translate(width/2, height/2, 0);
  
  // Display the text values on screen
  fill(0);
  textSize(24);
  text("Roll: " + nf(roll, 0, 2) + "°", -width/2 + 20, -height/2 + 40);
  text("Pitch: " + nf(pitch, 0, 2) + "°", -width/2 + 20, -height/2 + 70);
  text("Yaw: " + nf(yaw, 0, 2) + "°", -width/2 + 20, -height/2 + 100);
  
  // Rotate the 3D space based on your ESP32 data!
  // Processing uses Radians, so we convert your Degrees.
  // We negate or map axes depending on how the flight controller is oriented.
  rotateX(radians(-pitch));
  rotateZ(radians(roll));
  rotateY(radians(-yaw));
  
  // ==========================================
  // DRAWING THE DRONE
  // ==========================================
  
  // Draw the main body (Flight Controller)
  fill(50, 150, 250); 
  box(100, 20, 150);
  
  // Indicate the FRONT of the drone
  pushMatrix();
  translate(0, -11, 75);
  fill(250, 50, 50); // Red dot for the front
  sphere(10);
  popMatrix();
  
  // Draw Arms
  fill(100);
  
  // Front Right Arm
  pushMatrix();
  translate(50, 0, 75);
  rotateY(PI/4);
  box(120, 10, 20);
  popMatrix();
  
  // Front Left Arm
  pushMatrix();
  translate(-50, 0, 75);
  rotateY(-PI/4);
  box(120, 10, 20);
  popMatrix();
  
  // Back Right Arm
  pushMatrix();
  translate(50, 0, -75);
  rotateY(-PI/4);
  box(120, 10, 20);
  popMatrix();
  
  // Back Left Arm
  pushMatrix();
  translate(-50, 0, -75);
  rotateY(PI/4);
  box(120, 10, 20);
  popMatrix();
}

void serialEvent(Serial p) {
  data = p.readStringUntil('\n');
  if (data != null) {
    data = data.trim();
    println("Received: " + data); // Print to console for debugging
    
    if(data.startsWith("Accelerometer:")) {
      println("WARNING: You are running MPU6050_test.ino! Please upload FlightController.ino instead to see 3D rotation!");
    }
    
    // We are looking for the exact format your FlightController.ino prints:
    // "Roll : 12.34 °   Pitch : 45.67 °   Yaw : 89.01 °"
    if(data.startsWith("Roll :")) {
      // Remove degree symbols completely to fix Windows text encoding issues
      String cleanData = data.replace("°", ""); 
      
      // Split by spaces and colons
      String[] parts = splitTokens(cleanData, " :");
      
      // After splitting: [0]=Roll, [1]=12.34, [2]=Pitch, [3]=45.67, [4]=Yaw, [5]=89.01
      if(parts.length >= 6) {
        try {
          roll = float(parts[1]);
          pitch = float(parts[3]);
          yaw = float(parts[5]);
        } catch(Exception e) {
          println("Error parsing values!");
        }
      }
    }
  }
}
