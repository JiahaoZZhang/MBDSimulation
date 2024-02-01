#!/usr/bin/env python

# reference variable to Storyboard Omnet++ module: board
import storyboard
import timeline

print ("demo.py successfully imported...")

def createStories(board):

    # Create TimeCondition
    cond0 = storyboard.TimeCondition(timeline.seconds(0))

    # Create CarSetCondition
    cond1 = storyboard.CarSetCondition({"ego_1", "Obj"})

    and1 = storyboard.AndCondition(cond0, cond1)
    effect1 = storyboard.SpeedEffect(113)
    story = storyboard.Story(and1, [effect1])

    # Register Stories at the Storyboard
    # board.registerStory(story)

    print("Stories loaded!")

