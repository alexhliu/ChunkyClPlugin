package chunkycl;

import org.apache.commons.math3.util.FastMath;

/**
 * A packed quadtree implementation inspired by
 * https://stackoverflow.com/questions/41946007/efficient-and-well-explained-implementation-of-a-quadtree-for-2d-collision-det#answer-48330314
 * and the PackedOctree in Chunky
 */
public class PackedQuadtree {
    private static final int MAX_ARRAY_SIZE = Integer.MAX_VALUE - 16;
    private static final int DEFAULT_INITIAL_SIZE = 64;
    private static final double ARRAY_RESIZE_MULTIPLIER = 1.5;
    private static final int ANY_TYPE = 0x7FFFFFFE;

    public int[] treeData;

    private int size;
    private int freeHead;
    private int depth;

    public static class QuadtreeTooBigException extends RuntimeException {
    }

    public PackedQuadtree(int depth, long nodeCount) {
        this.depth = depth;

        if (nodeCount > MAX_ARRAY_SIZE)
            throw new QuadtreeTooBigException();

        treeData = new int[(int) FastMath.max(nodeCount, DEFAULT_INITIAL_SIZE)];

        treeData[0] = 0;
        size = 1;

        freeHead = -1;
    }

    public PackedQuadtree(int depth) {
        this.depth = depth;
        treeData = new int[DEFAULT_INITIAL_SIZE];

        treeData[0] = 0;
        size = 1;

        freeHead = -1;
    }

    private int findSpace() {
        if (freeHead != -1) {
            int index = freeHead;
            freeHead = treeData[freeHead];
            return index;
        }

        if (size + 4 <= treeData.length) {
            int index = size;
            size += 4;
            return index;
        }

        long newSize = (long) FastMath.ceil(treeData.length * ARRAY_RESIZE_MULTIPLIER);
        if (newSize > MAX_ARRAY_SIZE) {
            if (MAX_ARRAY_SIZE - size > 4) {
                newSize = MAX_ARRAY_SIZE;
            } else {
                throw new QuadtreeTooBigException();
            }
        }

        int[] newArray = new int[(int) newSize];
        System.arraycopy(treeData, 0, newArray, 0, size);
        treeData = newArray;

        int index = size;
        size += 4;
        return index;
    }

    private void freeSpace(int index) {
        treeData[index] = freeHead;
        freeHead = index;
    }

    private void subdivideNode(int nodeIndex) {
        int firstChildIndex = findSpace();

        for (int i = 0; i < 4; i++) {
            treeData[firstChildIndex + i] = treeData[nodeIndex];
        }

        treeData[nodeIndex] = firstChildIndex;
    }

    private void mergeNode(int nodeIndex, int typeNegation) {
        int childrenIndex = treeData[nodeIndex];
        freeSpace(childrenIndex);
        treeData[nodeIndex] = typeNegation;
    }

    private boolean nodeEquals(int firstNodeIndex, int secondNodeIndex) {
        boolean firstIsBranch = treeData[firstNodeIndex] > 0;
        boolean secondIsBranch = treeData[secondNodeIndex] > 0;
        return ((firstIsBranch && secondIsBranch) || treeData[firstNodeIndex] == treeData[secondNodeIndex]);
    }

    private boolean nodeDataEquals(int firstNodeIndex, int data) {
        boolean firstIsBranch = treeData[firstNodeIndex] > 0;
        return (firstIsBranch || treeData[firstNodeIndex] == -data);
    }

    public void set(int data, int x, int z) {
        int[] parents = new int[depth];
        int nodeIndex = 0;
        int position;

        for (int i = depth-1; i >= 0; i--) {
            parents[i] = nodeIndex;

            if (nodeDataEquals(nodeIndex, data)) {
                return;
            }

            if (treeData[nodeIndex] <= 0) {
                subdivideNode(nodeIndex);
            }

            int xbit = 1 & (x >> i);
            int zbit = 1 & (z >> i);
            position = (xbit << 1) | zbit;
            nodeIndex = treeData[nodeIndex] + position;
        }
        treeData[nodeIndex] = -data;

        for (int i = 0; i < depth; i++) {
            int parentIndex = parents[i];

            boolean allSame = true;
            for (int j = 0; j < 4; j++) {
                int childIndex = treeData[parentIndex] + j;
                if (!nodeEquals(childIndex, nodeIndex)) {
                    allSame = false;
                    break;
                }
            }

            if (allSame) {
                mergeNode(parentIndex, treeData[nodeIndex]);
            } else {
                break;
            }
        }
    }

    public void endFinalization() {
        finalizationNode(0);
    }

    private void finalizationNode(int nodeIndex) {
        boolean isStillMergeable = true;
        int mergedType = -ANY_TYPE;

        for (int i = 0; i < 4; i++) {
            int childIndex = treeData[nodeIndex] + i;

            if (treeData[childIndex] > 0) {
                finalizationNode(childIndex);

                if (treeData[childIndex] > 0) {
                    isStillMergeable = false;
                }
            }

            if (isStillMergeable) {
                if (mergedType == -ANY_TYPE) {
                    mergedType = treeData[childIndex];
                } else if (!(treeData[childIndex] == -ANY_TYPE || treeData[childIndex] == mergedType)) {
                    isStillMergeable = false;
                }
            }
        }

        if (isStillMergeable) {
            mergeNode(nodeIndex, mergedType);
        }
    }
}
