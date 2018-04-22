class StartingLeft():
    def __init__(self):
        self.startingPosition = "Left"

    def getStartingPosition(self):
        return self.startingPosition


class StartingMiddle():
    def __init__(self):
        self.startingPosition = "Middle"

    def getStartingPosition(self):
        return self.startingPosition


class StartingRight():
    def __init__(self):
        self.startingPosition = "Right"

    def getStartingPosition(self):
        return self.startingPosition


class ScoreScale():
    def __init__(self):
        self.scoringElement = "Scale"

    def getScoringElement(self):
        return self.scoringElement


class ScoreSwitch():
    def __init__(self):
        self.scoringElement = "Switch"

    def getScoringElement(self):
        return self.scoringElement


class CrossFieldEnable():
    def __init__(self):
        self.crossFieldEnable = True

    def getCrossFieldEnable(self):
        return self.crossFieldEnable


class CrossFieldDisable():
    def __init__(self):
        self.crossFieldEnable = False

    def getCrossFieldEnable(self):
        return self.crossFieldEnable
