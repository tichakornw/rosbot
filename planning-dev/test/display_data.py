import matplotlib.pyplot as plt
import pickle
import os

def readDataFromOneFile(filename):
    rawData = []
    with open(filename, 'r') as file:
        line = file.readline().strip()
        if (len(line) > 0):
            rawData.append(line)
        while line:
            line = file.readline()
            if (len(line) > 0):
                rawData.append(line)
    return rawData


class AverageData:
    def __init__(self, name, rawData):
        self.name = name
        self.dataX = [int(i.split(',')[0].strip()) for i in rawData]
        self.dataY = [float(i.split(',')[1].strip()) for i in rawData]


def readDataFromAllFiles():
    folders = [
        "iter_benchmark_data",
        "iter_op1_benchmark_data",
    ]

    foldersToName = {
        "iter_benchmark_data"       : "IDP",
        "iter_op1_benchmark_data"   : "IDP optimized BFS",
    }

    dataFilename = "average_data"
    averageData = []

    for f in folders:
        filename = os.path.join(f, dataFilename)
        rawData = readDataFromOneFile(filename)
        if (len(rawData) > 0):
            averageData.append(AverageData(foldersToName[f], rawData))

    return averageData


def displayData():
    fig, ax = plt.subplots()

    averageData = readDataFromAllFiles()
    for data in averageData:
        ax.plot(data.dataX, data.dataY, label=data.name, linewidth=1.0)

    ax.set_title("IDP Algorithm with 2 Monoids and Square Grid")
    ax.legend(title='Algorithms')
    ax.set_ylabel('Time In Seconds')
    ax.set_xlabel('Square Grid Side Length')
    plt.show()


def readMulticostAllocatedNodesDataFromAllFiles():
    folders = [
        "iter_op1_benchmark_data",
    ]

    nodeData = []
    multicostData = []

    for f in folders:
        filename = os.path.join(f, "nodes")
        rawData = readDataFromOneFile(filename)
        if (len(rawData) > 0):
            nodeData.append(AverageData("Number of Nodes Explored", rawData))
    
    for f in folders:
        filename = os.path.join(f, "multicost_allocated")
        rawData = readDataFromOneFile(filename)
        if (len(rawData) > 0):
            multicostData.append(AverageData("Number of Multicost Allocated", rawData))

    return nodeData, multicostData


def displayMulticostAllocatedNodesData():
    fig, ax = plt.subplots()

    nodeData, multicostData = readMulticostAllocatedNodesDataFromAllFiles()

    for data in nodeData:
        ax.plot(data.dataX, data.dataY, label=data.name, linewidth=1.0)

    ax.set_title("Number of Nodes Explored in Optimal Subgraph Algorithm")
    ax.set_ylabel('Number of Nodes')
    ax.set_xlabel('Square Grid Side Length')
    plt.show()

    fig, ax = plt.subplots()

    for data in multicostData:
        ax.plot(data.dataX, data.dataY, label=data.name, linewidth=1.0)

    ax.set_title("Multicost Allocated in Optimal Subgraph Algorithm")
    ax.set_ylabel('Number of Multicost Allocated')
    ax.set_xlabel('Square Grid Side Length')
    plt.show()



def displayTestEnvironment():
    graphFP = "cached_benchmark_data/gridgraph_1000.pkl"

    with open(graphFP, 'rb') as f:
        graphImageArray = pickle.load(f)
        #graphImageArray = [[1 if i == 0 else 0 for i in j] for j in graphImageArray]
        plt.imshow(graphImageArray, cmap='gray', vmin=0, vmax=1)
        plt.title("Example Grid Environment")
        plt.show()


if __name__ == "__main__":
    #displayData()
    #displayTestEnvironment()
    displayMulticostAllocatedNodesData()