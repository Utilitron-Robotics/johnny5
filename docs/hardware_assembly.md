# Assembly

## Mobile Base Assembly

1. Using the servo software ([FD Debug Tool v1.9.8.3](https://www.feetechrc.com/Data/feetechrc/upload/file/20240622/FD1.9.8.3.zip)), assign ID numbers 8, 9, 10, and 11 to the servos respectively.

   **How to adjust:**
   
   To assign ID numbers to the servos using the FD Debug Tool:
   
   - **Download and install** the FD Debug Tool from the link above
   - **Connect** the servo to your computer using the Waveshare Bus Servo Controller (via USB) 
   - **Launch** the FD Debug Tool software
   - **Select** the correct COM port and set the baud rate (typically 1000000 or as specified in your servo manual)
   - **Click** "Scan Servo" to detect the connected servo and its current ID
   - **Change the ID** by entering the desired number (8, 9, 10, or 11) in the ID field
   - **Click** "Write" or "Save" to update the servo's ID
   - **Disconnect** the servo and repeat the process for each remaining servo
   
   **Note:** Make sure to connect and configure only one servo at a time to avoid ID conflicts during the setup process.


1. Connect servos 8, 9, and 10 using the included 20cm servo cables, and connect servos 10 and 11 using a 90cm servo cable.
<img src="./media/assembly/1.jpg" width="400">

3. Install M3 heat-set inserts into the servo brackets using a soldering iron.
<img src="./media/assembly/2.jpg" width="400">

4. Secure the servos using the small screws included with the servos.
<img src="./media/assembly/4.jpg" width="400">

5. Install the wheel axle connectors.
<img src="./media/assembly/5.jpg" width="400">
If the connector doesn't fit, sand it down for a better fit.
<img src="./media/assembly/6.jpg" width="400">
Connector installation complete (if it's loose, tighten with screws or use epoxy resin for bonding; no additional treatment needed if it's tight enough).
<img src="./media/assembly/8.jpg" width="400">

6. Mount the servos to the chassis plate.
<img src="./media/assembly/9.jpg" width="400">
Follow the servo order as shown in the image.
<img src="./media/assembly/10.jpg" width="400">

7. Secure the servos to the chassis plate using M3x14 screws.
<img src="./media/assembly/11.jpg" width="400">

8. Attach the omni wheels to the servo connector using M3x18 screws.
<img src="./media/assembly/12.jpg" width="400">
<img src="./media/assembly/13.jpg" width="400">

9.  (optional)Install the chassis bearings.
Insert the bearings (12x18x4) into the dowel pin and shaft sleeves.
<img src="./media/assembly/14.jpg" width="400">
<img src="./media/assembly/15.jpg" width="400">
<img src="./media/assembly/16.jpg" width="400">

10. Install M3 heat-set inserts into the side plates.
<img src="./media/assembly/17.jpg" width="400">

11. Assemble the main posts like building blocks - there are 4 in total. Use double-sided tape or epoxy resin to bond the parts together.
<img src="./media/assembly/18.jpg" width="400">

12. Install the lift axis. Prepare the materials according to the BOM list.
<img src="./media/assembly/19.jpg" width="400">

13. Secure the gear using the screws included with the servo.
<img src="./media/assembly/20.jpg" width="400">

14. Insert the gear into the Axis_Servo_Mount.
<img src="./media/assembly/21.jpg" width="400">

15. Secure the Axis_Bracket to the servo using the included M3x6 screws from the servo.
<img src="./media/assembly/22.jpg" width="400">


17. Install the T-bracket.
<img src="./media/assembly/24.jpg" width="400">

18. Secure it with double-sided tape or epoxy resin.
<img src="./media/assembly/25.jpg" width="400">
<img src="./media/assembly/26.jpg" width="400">
<img src="./media/assembly/27.jpg" width="400">

19. Slide the lift axis into the T-bracket.
<img src="./media/assembly/28.jpg" width="400">
<img src="./media/assembly/29.jpg" width="400">

20. Install the Top and Back cameras.
Remove the two screws from the camera back cover.
<img src="./media/assembly/30.jpg" width="400">

21. Reinstall the back cover using M2x12 screws, sandwiching the printed part in between.
<img src="./media/assembly/31.jpg" width="400">

22. Route the camera cable through the internal channel of the printed part.
<img src="./media/assembly/32.jpg" width="400">

23. Secure the display to the printed part using the included M3 servo screws.
<img src="./media/assembly/34.jpg" width="400">

24. Slide the display into the rear slot of the main post.
<img src="./media/assembly/35.jpg" width="400">

25. Install the Front camera using the same method as the Top camera.
Slide it into the slot. If there are any burrs on the printed part, file them down for a smooth fit.

26. Route the USB Type-C data cable and DC power cable through the hole on the rear side of the main post.
<img src="./media/assembly/33.jpg" width="400">

27. Mobile Base assembly complete!
<img src="./media/assembly/37.jpg" width="400">

## Leader arms and Follower arms
For the assembly of Leader arms and Follower arms, please refer to the tutorial: [SO-101 Assembly Guide](https://huggingface.co/docs/lerobot/so101). 


## Follower arms and Mobile Base Assembly

1. Secure the robotic arm to both sides of the T-bracket using four M3x30 hex screws and four M3 nuts.
   <img src="./media/assembly/38.jpg" width="400">

2. Connect the 90cm servo cable from servo #11 to the other port on the Waveshare controller board of the **left arm**.
   <img src="./media/assembly/39.jpg" width="400">

3. Attach the 12V to 5V step-down converter to the base and connect it to the Raspberry Pi.
   <img src="./media/assembly/40.jpg" width="400">

4. Connect the 12V lithium battery to the step-down converter to provide power.
   <img src="./media/assembly/41.jpg" width="400">

5. Connect the male plug of the 1-to-2 DC splitter cable to the female connectors of the two DC extension cables.
   <img src="./media/assembly/42.jpg" width="400">

6. Connect the second 12V lithium battery to the female connector of the 1-to-2 DC splitter cable.
   <img src="./media/assembly/43.jpg" width="400">

7. Assembly fully complete!
   <img src="./media/alohamini_git.png" >

