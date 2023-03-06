Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

**1. How much current does the system draw (instantaneous measurement) when a single LED is on with the GPIO pin set to StrongAlternateStrong?**
   Answer: When a single LED is on with the GPIO pin being set to StrongAlternateStrong, current drawn by the system is 5.28 mA


**2. How much current does the system draw (instantaneous measurement) when a single LED is on with the GPIO pin set to WeakAlternateWeak?**
   Answer: When a single LED is on with the GPIO pin being set to WeakAlternateWeak, current drawn by the system is 5.33 mA


**3. Is there a meaningful difference in current between the answers for question 1 and 2? Please explain your answer, 
referencing the [Mainboard Schematic](https://www.silabs.com/documents/public/schematic-files/WSTK-Main-BRD4001A-A01-schematic.pdf) and [AEM Accuracy](https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf) section of the user's guide where appropriate. Extra credit is avilable for this question and depends on your answer.**
   Answer: No, There is no difference in current between the WeakAlternateWeak and StrongAlternateStrong setting. Because of the following reason.
    When all the LEDs are turned Off and if the current drawn from the system is measured in the AEM, It is coming around 4.72 mA. So which means LED is drawing around 0.65mA of current approximately. According to the drive strength for Weak and Strong attribute setting, Weak setting gives 1mA drive current where as Strong setting gives around 10mA of drive current. Since LED draws only 0.65 mA of current, There is no significance of this setting. 
    


**4. With the WeakAlternateWeak drive strength setting, what is the average current for 1 complete on-off cycle for 1 LED with an on-off duty cycle of 50% (approximately 1 sec on, 1 sec off)?**
   Answer: Average current for 1 complete on-off cycle for 1 LED with the WeakAlternateWeak drive strength setting is 5.14 mA
   (50 % duty cycle).


**5. With the WeakAlternateWeak drive strength setting, what is the average current for 1 complete on-off cycle for 2 LEDs (both on at the time same and both off at the same time) with an on-off duty cycle of 50% (approximately 1 sec on, 1 sec off)?**
   Answer: Average current for 1 complete on-off cycle for 2 LEDs with the WeakAlternateWeak drive strength setting is 5.25 mA (50 % duty cycle).


