#-------------------------- PARAMETERS --------------------------#
maxBoardVoltage = 3.3
maxBatteryVoltage = 17
outputCount = 15
# Desired R2/(R1+R2):
desiredValue = maxBoardVoltage / maxBatteryVoltage
# Include all available sizes of resistor:
availableResistances = [10, 47, 100, 150, 220, 330, 470, 1000, 1200, 1500, 2200, 3300, 4700, 10000]
# Recommended: Choose the pair that balances a high ratio less than the desired value with a high totalR.
#----------------------------------------------------------------#

# Represents one possible pairing of resistors
class ResistorGroup:
    def __init__(self, r1, r2):
        self.r1 = r1
        self.r2 = r2
        self.totalR = r1 + r2
        self.ratio = r2 / self.totalR
        self.error = abs(self.ratio - desiredValue)
        
    def __repr__(self):
        return f"r1={self.r1}, r2={self.r2}, totalR={self.totalR}, ratio={self.ratio}, error={self.error})"

# Try all pairs and sort by error
groups = []
for r1 in availableResistances:
    for r2 in availableResistances:
        groups.append(ResistorGroup(r1, r2))
groupsSortedByError = sorted(groups, key=lambda group: group.error)

# Print desired value and best outputCount choices
print(f"Desired Value: {desiredValue}")
for i in range(outputCount):
    print(groupsSortedByError[i])