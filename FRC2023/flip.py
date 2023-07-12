import json
arr = ['Blue Bottom Mid Pickup']

for s in arr:
  # Opening JSON file
  f = open(s+'.path')

  def bound(angle):
      while (angle >= 180.0):
        angle -= 360.0
      
      while (angle < -180.0):
        angle += 360.0
      
      return angle

    
  # returns JSON object as 
  # a dictionary
  '''
  "anchorPoint": {
          "x": 4.845447314490403,
          "y": 4.028355964242929
        },
        "prevControl": null,
        "nextControl": {
          "x": 4.886619710733237,
          "y": 4.028355964242929
        },
        "holonomicAngle": 0,
  '''
  data = json.load(f)
    
  # Iterating through the json
  # list
  for i in data['waypoints']:
      anc = i['anchorPoint']
      prev = i['prevControl']
      nxt = i['nextControl']
      angle = i["holonomicAngle"]

      anc['x'] = 16.54-anc['x']
      angle = -bound(angle+180.)
      i["holonomicAngle"] = angle
      if prev != None:
          prev['x'] = 16.54-prev['x']
      if nxt != None:
          nxt['x'] = 16.54-nxt['x']

  with open(s.replace('Blue','Red')+'.path', "w") as f:
      json.dump(data, f)

  # Closing file
  f.close()