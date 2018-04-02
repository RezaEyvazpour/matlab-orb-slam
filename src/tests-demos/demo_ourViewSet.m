vs = ourViewSet()

vs = vs.addView(1,1)
vs = vs.addView(2,2)
vs = vs.addView(3,3)

vs = vs.addConnection(1,2)
vs = vs.addConnection(2,3)
vs = vs.addConnection(1,3)

vs = vs.deleteConnection(1,2)
vs = vs.addConnection(1,2)

vs = vs.deleteView(2)