# Clock Final Project

## Clock Specifications

1) The clock must display the date and time. How you choose to do this is up to
you. Using the 7 segment displays, you will need to show the day of the week, the
month, the date, and the year. You will also need to show the current time. I
recommend you set up the clock to run-in 24-hour mode vs. AM/PM, but you may
do whichever mode you prefer. Options for showing this information includes using
the marquee mode for the display and/or cycling through different “pages”.

2) You must have a mode for setting the date and time. You may use any
combination of buttons, switches, and knobs that you choose to implement this. It
should be clear how to set the values when in this mode.

3) You must have a mode for setting the Alarm time. You may use any
combination of buttons, switches, and knobs that you choose to implement this. It
should be clear how to set the values when in this mode.

4) You must have a means of enabling or disabling the alarm. When the alarm is
enabled, it is helpful to periodically show the time that the alarm is set to go off.

6) The Alarm interrupt is tricky to setup, so I recommend that you use the Systik
handler instead and check bit 8 in the RTC_ISR register to see when the alarm has
been triggered, and then play a song or other sounds to implement the alarm. Once
you start playing the alarm, you should clear bit 8 back to a zero.

## Deliverables

1) Submit your C program to Canvas as PDF or TXT. Be sure to comment your
code or you will be docked points! This submittal is a team effort.

2) Include a paragraph in your report discussing what you felt was the most
challenging part of your project and how you met this challenge.
3) Obtain check-off by showing the operation of your program to your instructor or
lab assistant.

4) Individually answer the three ABET questions found in the Assignments section
in Canvas. You will get points for answering these questions.
