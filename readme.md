##### LEOC - Low Earth orbit cicerone (UrGraphMedVeh)

pip3 install -U spacetrack pyorbital deprecation pyqtgraph ntplib opencv-python

To get rid off pyqtgraph's ignored exception, add line to site-packages/pyqtgraph/graphicsItems/ScatterPlotItem.py:

    795 line: if source_rect[viewMask].tolist() and source_rect[viewMask].tolist()[0]:
    
Create logs/ folder