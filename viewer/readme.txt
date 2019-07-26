Instructions

Generate the viewer file
1. In AF3: 
    -> Open the model you want to use
    -> Select the node "Allocations" 
    -> Select the subnode "Components -> Hardware"
    -> Right-click and select "Export for SystemFocus viewer"
2. Save the file as 'ff1.sf1viewer'
3. Connect your PC to the router 'FF1RoadNet'
4. Open the file 'ff1.sf1viewer' generated above: 
    -> Search for the entry with key "server.ip" 
    -> Here replace the ip of the server with the one of your PC

Start the viewer
1. Unzip the folder corresponding to your operating system, e.g., 'org.fortiss.sf1.af3.viewer.product-win32.win32.x86_64'
2. Run the 'eclipse' application contained in the main folder
3. Once the viewer is open, right-click and select 'Load...' 
    -> Then browse and select the file 'ff1.sf1viewer'
4. Right-click again and select 'Start Server'

Start the rover
1. Connect to the rover as usual
2. Edit the file 'adaptBlack.sh', e.g., by typing 'nano adaptBlack.sh'
    -> Modify the value of the variable 'IP_CC_DEVICE' by setting it to the ip of your PC 
4. Enter the folder 'SS19' by typing 'cd SS19'
5. Run './makeComplete'
6. Run './build/RaspberryPI.run'

---

Note

In order to run the application, it might be necessary to adjust some firewall options on your operating system.
For Windows Firewall, the following worked on our machine:
    - Open 'Windows Firewall with Advanced Security'
    - Select 'Inbound Rules'
    - In the group 'File and Printer Sharing', enable 'File and Printer Sharing (Echo Request - ICMP4-In)' --- you might have more than one item with such a name