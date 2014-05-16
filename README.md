TRAC3R - MOC LEGO Mindstorms EV3 drawing robot arm
------

See the video of TRAC3R in action: http://youtu.be/r8s6GkPP5EM

To use TRAC3R, do not forget to include the SVG Salamander 
(https://svgsalamander.java.net) SVG engine for Java to the things you transfer to EV3 and
add it to the libraries path in your ant file. The "tiny" version is good enough.

Currently TRAC3R expects a SVG file at "/tmp/job.svg".

SVG files work best if they consist of thin lines. I played with inkscape to draw a svg
and with potrace to convert a picture into a svg.


A few words about the ldd file: I was not able to put the last 12 tooth bevel gear to the
model. It seems that it fits there tight in real live -  a little too tight for ldd. Maybe
the way the first joint is moved has still room for improvement.

Things that I think would be cool to achieve with TRAC3R:
* Make it draw more accurately
* Make drawing movements more fluid
* PC / Mac App to create and send drawing jobs via network. Maybe "realtime" drawing 
  controlled by iPad or the like:-)

Have fun an please show me what you have done with TRAC3R!

If you have improvements for the code or model, please let me know!

Cheers,
Christoph


