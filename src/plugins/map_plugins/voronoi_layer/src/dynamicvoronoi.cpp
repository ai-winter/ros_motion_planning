#include "dynamicvoronoi.h"

#include <math.h>
#include <iostream>

DynamicVoronoi::DynamicVoronoi()
{
  sqrt2 = sqrt(2.0);
  data = NULL;
  gridMap = NULL;
  alternativeDiagram = NULL;
  allocatedGridMap = false;
}

DynamicVoronoi::~DynamicVoronoi()
{
  if (data)
  {
    for (int x = 0; x < sizeX; x++)
      delete[] data[x];
    delete[] data;
  }
  if (allocatedGridMap && gridMap)
  {
    for (int x = 0; x < sizeX; x++)
      delete[] gridMap[x];
    delete[] gridMap;
  }
}

void DynamicVoronoi::initializeEmpty(int _sizeX, int _sizeY, bool initGridMap)
{
  if (data)
  {
    for (int x = 0; x < sizeX; x++)
      delete[] data[x];
    delete[] data;
    data = NULL;
  }
  if (alternativeDiagram)
  {
    for (int x = 0; x < sizeX; x++)
      delete[] alternativeDiagram[x];
    delete[] alternativeDiagram;
    alternativeDiagram = NULL;
  }
  if (initGridMap)
  {
    if (allocatedGridMap && gridMap)
    {
      for (int x = 0; x < sizeX; x++)
        delete[] gridMap[x];
      delete[] gridMap;
      gridMap = NULL;
      allocatedGridMap = false;
    }
  }

  sizeX = _sizeX;
  sizeY = _sizeY;
  data = new dataCell*[sizeX];
  for (int x = 0; x < sizeX; x++)
    data[x] = new dataCell[sizeY];

  if (initGridMap)
  {
    gridMap = new bool*[sizeX];
    for (int x = 0; x < sizeX; x++)
      gridMap[x] = new bool[sizeY];
    allocatedGridMap = true;
  }

  dataCell c;
  c.dist = INFINITY;
  c.sqdist = INT_MAX;
  c.obstX = invalidObstData;
  c.obstY = invalidObstData;
  c.voronoi = free;
  c.queueing = fwNotQueued;
  c.needsRaise = false;

  for (int x = 0; x < sizeX; x++)
    for (int y = 0; y < sizeY; y++)
      data[x][y] = c;

  if (initGridMap)
  {
    for (int x = 0; x < sizeX; x++)
      for (int y = 0; y < sizeY; y++)
        gridMap[x][y] = 0;
  }
}

void DynamicVoronoi::initializeMap(int _sizeX, int _sizeY, bool** _gridMap)
{
  gridMap = _gridMap;
  initializeEmpty(_sizeX, _sizeY, false);

  for (int x = 0; x < sizeX; x++)
  {
    for (int y = 0; y < sizeY; y++)
    {
      if (gridMap[x][y])
      {
        dataCell c = data[x][y];
        if (!isOccupied(x, y, c))
        {
          bool isSurrounded = true;
          for (int dx = -1; dx <= 1; dx++)
          {
            int nx = x + dx;
            if (nx <= 0 || nx >= sizeX - 1)
              continue;
            for (int dy = -1; dy <= 1; dy++)
            {
              if (dx == 0 && dy == 0)
                continue;
              int ny = y + dy;
              if (ny <= 0 || ny >= sizeY - 1)
                continue;

              if (!gridMap[nx][ny])
              {
                isSurrounded = false;
                break;
              }
            }
          }
          if (isSurrounded)
          {
            c.obstX = x;
            c.obstY = y;
            c.sqdist = 0;
            c.dist = 0;
            c.voronoi = occupied;
            c.queueing = fwProcessed;
            data[x][y] = c;
          }
          else
            setObstacle(x, y);
        }
      }
    }
  }
}

void DynamicVoronoi::occupyCell(int x, int y)
{
  gridMap[x][y] = 1;
  setObstacle(x, y);
}
void DynamicVoronoi::clearCell(int x, int y)
{
  gridMap[x][y] = 0;
  removeObstacle(x, y);
}

void DynamicVoronoi::setObstacle(int x, int y)
{
  dataCell c = data[x][y];
  if (isOccupied(x, y, c))
    return;

  addList.push_back(INTPOINT(x, y));
  c.obstX = x;
  c.obstY = y;
  data[x][y] = c;
}

void DynamicVoronoi::removeObstacle(int x, int y)
{
  dataCell c = data[x][y];
  if (isOccupied(x, y, c) == false)
    return;

  removeList.push_back(INTPOINT(x, y));
  c.obstX = invalidObstData;
  c.obstY = invalidObstData;
  c.queueing = bwQueued;
  data[x][y] = c;
}

void DynamicVoronoi::exchangeObstacles(std::vector<INTPOINT>& points)
{
  for (unsigned int i = 0; i < lastObstacles.size(); i++)
  {
    int x = lastObstacles[i].x;
    int y = lastObstacles[i].y;

    bool v = gridMap[x][y];
    if (v)
      continue;
    removeObstacle(x, y);
  }

  lastObstacles.clear();

  for (unsigned int i = 0; i < points.size(); i++)
  {
    int x = points[i].x;
    int y = points[i].y;
    bool v = gridMap[x][y];
    if (v)
      continue;
    setObstacle(x, y);
    lastObstacles.push_back(points[i]);
  }
}

void DynamicVoronoi::update(bool updateRealDist)
{
  commitAndColorize(updateRealDist);

  while (!open.empty())
  {
    INTPOINT p = open.pop();
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if (c.queueing == fwProcessed)
      continue;

    if (c.needsRaise)
    {
      // RAISE
      for (int dx = -1; dx <= 1; dx++)
      {
        int nx = x + dx;
        if (nx <= 0 || nx >= sizeX - 1)
          continue;
        for (int dy = -1; dy <= 1; dy++)
        {
          if (dx == 0 && dy == 0)
            continue;
          int ny = y + dy;
          if (ny <= 0 || ny >= sizeY - 1)
            continue;
          dataCell nc = data[nx][ny];
          if (nc.obstX != invalidObstData && !nc.needsRaise)
          {
            if (!isOccupied(nc.obstX, nc.obstY, data[nc.obstX][nc.obstY]))
            {
              open.push(nc.sqdist, INTPOINT(nx, ny));
              nc.queueing = fwQueued;
              nc.needsRaise = true;
              nc.obstX = invalidObstData;
              nc.obstY = invalidObstData;
              if (updateRealDist)
                nc.dist = INFINITY;
              nc.sqdist = INT_MAX;
              data[nx][ny] = nc;
            }
            else
            {
              if (nc.queueing != fwQueued)
              {
                open.push(nc.sqdist, INTPOINT(nx, ny));
                nc.queueing = fwQueued;
                data[nx][ny] = nc;
              }
            }
          }
        }
      }
      c.needsRaise = false;
      c.queueing = bwProcessed;
      data[x][y] = c;
    }
    else if (c.obstX != invalidObstData && isOccupied(c.obstX, c.obstY, data[c.obstX][c.obstY]))
    {
      // LOWER
      c.queueing = fwProcessed;
      c.voronoi = occupied;

      for (int dx = -1; dx <= 1; dx++)
      {
        int nx = x + dx;
        if (nx <= 0 || nx >= sizeX - 1)
          continue;
        for (int dy = -1; dy <= 1; dy++)
        {
          if (dx == 0 && dy == 0)
            continue;
          int ny = y + dy;
          if (ny <= 0 || ny >= sizeY - 1)
            continue;
          dataCell nc = data[nx][ny];
          if (!nc.needsRaise)
          {
            int distx = nx - c.obstX;
            int disty = ny - c.obstY;
            int newSqDistance = distx * distx + disty * disty;
            bool overwrite = (newSqDistance < nc.sqdist);
            if (!overwrite && newSqDistance == nc.sqdist)
            {
              if (nc.obstX == invalidObstData || isOccupied(nc.obstX, nc.obstY, data[nc.obstX][nc.obstY]) == false)
                overwrite = true;
            }
            if (overwrite)
            {
              open.push(newSqDistance, INTPOINT(nx, ny));
              nc.queueing = fwQueued;
              if (updateRealDist)
              {
                nc.dist = sqrt((double)newSqDistance);
              }
              nc.sqdist = newSqDistance;
              nc.obstX = c.obstX;
              nc.obstY = c.obstY;
            }
            else
            {
              checkVoro(x, y, nx, ny, c, nc);
            }
            data[nx][ny] = nc;
          }
        }
      }
    }
    data[x][y] = c;
  }
}

float DynamicVoronoi::getDistance(int x, int y) const
{
  if ((x > 0) && (x < sizeX) && (y > 0) && (y < sizeY))
    return data[x][y].dist;
  else
    return -INFINITY;
}

int DynamicVoronoi::getObstacleX(int x, int y) const
{
  if ((x > 0) && (x < sizeX) && (y > 0) && (y < sizeY))
    return data[x][y].obstX;
  else
    return -1;
}

int DynamicVoronoi::getObstacleY(int x, int y) const
{
  if ((x > 0) && (x < sizeX) && (y > 0) && (y < sizeY))
    return data[x][y].obstY;
  else
    return -1;
}

bool DynamicVoronoi::isVoronoi(int x, int y) const
{
  dataCell c = data[x][y];
  return (c.voronoi == free || c.voronoi == voronoiKeep);
}

bool DynamicVoronoi::isVoronoiAlternative(int x, int y) const
{
  int v = alternativeDiagram[x][y];
  return (v == free || v == voronoiKeep);
}

void DynamicVoronoi::commitAndColorize(bool updateRealDist)
{
  // ADD NEW OBSTACLES
  for (unsigned int i = 0; i < addList.size(); i++)
  {
    INTPOINT p = addList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if (c.queueing != fwQueued)
    {
      if (updateRealDist)
        c.dist = 0;
      c.sqdist = 0;
      c.obstX = x;
      c.obstY = y;
      c.queueing = fwQueued;
      c.voronoi = occupied;
      data[x][y] = c;
      open.push(0, INTPOINT(x, y));
    }
  }

  // REMOVE OLD OBSTACLES
  for (unsigned int i = 0; i < removeList.size(); i++)
  {
    INTPOINT p = removeList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if (isOccupied(x, y, c) == true)
      continue;  // obstacle was removed and reinserted
    open.push(0, INTPOINT(x, y));
    if (updateRealDist)
      c.dist = INFINITY;
    c.sqdist = INT_MAX;
    c.needsRaise = true;
    data[x][y] = c;
  }
  removeList.clear();
  addList.clear();
}

void DynamicVoronoi::checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc)
{
  if ((c.sqdist > 1 || nc.sqdist > 1) && nc.obstX != invalidObstData)
  {
    if (abs(c.obstX - nc.obstX) > 1 || abs(c.obstY - nc.obstY) > 1)
    {
      // compute dist from x,y to obstacle of nx,ny
      int dxy_x = x - nc.obstX;
      int dxy_y = y - nc.obstY;
      int sqdxy = dxy_x * dxy_x + dxy_y * dxy_y;
      int stability_xy = sqdxy - c.sqdist;
      if (sqdxy - c.sqdist < 0)
        return;

      // compute dist from nx,ny to obstacle of x,y
      int dnxy_x = nx - c.obstX;
      int dnxy_y = ny - c.obstY;
      int sqdnxy = dnxy_x * dnxy_x + dnxy_y * dnxy_y;
      int stability_nxy = sqdnxy - nc.sqdist;
      if (sqdnxy - nc.sqdist < 0)
        return;

      // which cell is added to the Voronoi diagram?
      if (stability_xy <= stability_nxy && c.sqdist > 2)
      {
        if (c.voronoi != free)
        {
          c.voronoi = free;
          reviveVoroNeighbors(x, y);
          pruneQueue.push(INTPOINT(x, y));
        }
      }
      if (stability_nxy <= stability_xy && nc.sqdist > 2)
      {
        if (nc.voronoi != free)
        {
          nc.voronoi = free;
          reviveVoroNeighbors(nx, ny);
          pruneQueue.push(INTPOINT(nx, ny));
        }
      }
    }
  }
}

void DynamicVoronoi::reviveVoroNeighbors(int& x, int& y)
{
  for (int dx = -1; dx <= 1; dx++)
  {
    int nx = x + dx;
    if (nx <= 0 || nx >= sizeX - 1)
      continue;
    for (int dy = -1; dy <= 1; dy++)
    {
      if (dx == 0 && dy == 0)
        continue;
      int ny = y + dy;
      if (ny <= 0 || ny >= sizeY - 1)
        continue;
      dataCell nc = data[nx][ny];
      if (nc.sqdist != INT_MAX && !nc.needsRaise && (nc.voronoi == voronoiKeep || nc.voronoi == voronoiPrune))
      {
        nc.voronoi = free;
        data[nx][ny] = nc;
        pruneQueue.push(INTPOINT(nx, ny));
      }
    }
  }
}

bool DynamicVoronoi::isOccupied(int x, int y) const
{
  dataCell c = data[x][y];
  return (c.obstX == x && c.obstY == y);
}

bool DynamicVoronoi::isOccupied(int& x, int& y, dataCell& c)
{
  return (c.obstX == x && c.obstY == y);
}

void DynamicVoronoi::visualize(const char* filename)
{
  // write ppm files

  FILE* F = fopen(filename, "w");
  if (!F)
  {
    std::cerr << "could not open 'result.pgm' for writing!\n";
    return;
  }
  fprintf(F, "P6\n#\n");
  fprintf(F, "%d %d\n255\n", sizeX, sizeY);

  for (int y = sizeY - 1; y >= 0; y--)
  {
    for (int x = 0; x < sizeX; x++)
    {
      unsigned char c = 0;
      if (alternativeDiagram != NULL && (alternativeDiagram[x][y] == free || alternativeDiagram[x][y] == voronoiKeep))
      {
        fputc(255, F);
        fputc(0, F);
        fputc(0, F);
      }
      else if (isVoronoi(x, y))
      {
        fputc(0, F);
        fputc(0, F);
        fputc(255, F);
      }
      else if (data[x][y].sqdist == 0)
      {
        fputc(0, F);
        fputc(0, F);
        fputc(0, F);
      }
      else
      {
        float f = 80 + (sqrt(data[x][y].sqdist) * 10);
        if (f > 255)
          f = 255;
        if (f < 0)
          f = 0;
        c = (unsigned char)f;
        fputc(c, F);
        fputc(c, F);
        fputc(c, F);
      }
    }
  }
  fclose(F);
}

void DynamicVoronoi::prune()
{
  // filler
  while (!pruneQueue.empty())
  {
    INTPOINT p = pruneQueue.front();
    pruneQueue.pop();
    int x = p.x;
    int y = p.y;

    if (data[x][y].voronoi == occupied)
      continue;
    if (data[x][y].voronoi == freeQueued)
      continue;

    data[x][y].voronoi = freeQueued;
    sortedPruneQueue.push(data[x][y].sqdist, p);

    /* tl t tr
       l c r
       bl b br */

    dataCell tr, tl, br, bl;
    tr = data[x + 1][y + 1];
    tl = data[x - 1][y + 1];
    br = data[x + 1][y - 1];
    bl = data[x - 1][y - 1];

    dataCell r, b, t, l;
    r = data[x + 1][y];
    l = data[x - 1][y];
    t = data[x][y + 1];
    b = data[x][y - 1];

    if (x + 2 < sizeX && r.voronoi == occupied)
    {
      // fill to the right
      if (tr.voronoi != occupied && br.voronoi != occupied && data[x + 2][y].voronoi != occupied)
      {
        r.voronoi = freeQueued;
        sortedPruneQueue.push(r.sqdist, INTPOINT(x + 1, y));
        data[x + 1][y] = r;
      }
    }
    if (x - 2 >= 0 && l.voronoi == occupied)
    {
      // fill to the left
      if (tl.voronoi != occupied && bl.voronoi != occupied && data[x - 2][y].voronoi != occupied)
      {
        l.voronoi = freeQueued;
        sortedPruneQueue.push(l.sqdist, INTPOINT(x - 1, y));
        data[x - 1][y] = l;
      }
    }
    if (y + 2 < sizeY && t.voronoi == occupied)
    {
      // fill to the top
      if (tr.voronoi != occupied && tl.voronoi != occupied && data[x][y + 2].voronoi != occupied)
      {
        t.voronoi = freeQueued;
        sortedPruneQueue.push(t.sqdist, INTPOINT(x, y + 1));
        data[x][y + 1] = t;
      }
    }
    if (y - 2 >= 0 && b.voronoi == occupied)
    {
      // fill to the bottom
      if (br.voronoi != occupied && bl.voronoi != occupied && data[x][y - 2].voronoi != occupied)
      {
        b.voronoi = freeQueued;
        sortedPruneQueue.push(b.sqdist, INTPOINT(x, y - 1));
        data[x][y - 1] = b;
      }
    }
  }

  while (!sortedPruneQueue.empty())
  {
    INTPOINT p = sortedPruneQueue.pop();
    dataCell c = data[p.x][p.y];
    int v = c.voronoi;
    if (v != freeQueued && v != voronoiRetry)
    {  // || v>free || v==voronoiPrune || v==voronoiKeep) {
      //      assert(v!=retry);
      continue;
    }

    markerMatchResult r = markerMatch(p.x, p.y);
    if (r == pruned)
      c.voronoi = voronoiPrune;
    else if (r == keep)
      c.voronoi = voronoiKeep;
    else
    {  // r==retry
      c.voronoi = voronoiRetry;
      //      printf("RETRY %d %d\n", x, sizeY-1-y);
      pruneQueue.push(p);
    }
    data[p.x][p.y] = c;

    if (sortedPruneQueue.empty())
    {
      while (!pruneQueue.empty())
      {
        INTPOINT p = pruneQueue.front();
        pruneQueue.pop();
        sortedPruneQueue.push(data[p.x][p.y].sqdist, p);
      }
    }
  }
  //  printf("match: %d\nnomat: %d\n", matchCount, noMatchCount);
}

void DynamicVoronoi::updateAlternativePrunedDiagram()
{
  if (alternativeDiagram == NULL)
  {
    alternativeDiagram = new int*[sizeX];
    for (int x = 0; x < sizeX; x++)
    {
      alternativeDiagram[x] = new int[sizeY];
    }
  }

  std::queue<INTPOINT> end_cells;
  BucketPrioQueue<INTPOINT> sortedPruneQueue;
  for (int x = 1; x < sizeX - 1; x++)
  {
    for (int y = 1; y < sizeY - 1; y++)
    {
      dataCell& c = data[x][y];
      alternativeDiagram[x][y] = c.voronoi;
      if (c.voronoi <= free)
      {
        sortedPruneQueue.push(c.sqdist, INTPOINT(x, y));
        end_cells.push(INTPOINT(x, y));
      }
    }
  }

  for (int x = 1; x < sizeX - 1; x++)
  {
    for (int y = 1; y < sizeY - 1; y++)
    {
      if (getNumVoronoiNeighborsAlternative(x, y) >= 3)
      {
        alternativeDiagram[x][y] = voronoiKeep;
        sortedPruneQueue.push(data[x][y].sqdist, INTPOINT(x, y));
        end_cells.push(INTPOINT(x, y));
      }
    }
  }

  for (int x = 1; x < sizeX - 1; x++)
  {
    for (int y = 1; y < sizeY - 1; y++)
    {
      if (getNumVoronoiNeighborsAlternative(x, y) >= 3)
      {
        alternativeDiagram[x][y] = voronoiKeep;
        sortedPruneQueue.push(data[x][y].sqdist, INTPOINT(x, y));
        end_cells.push(INTPOINT(x, y));
      }
    }
  }

  while (!sortedPruneQueue.empty())
  {
    INTPOINT p = sortedPruneQueue.pop();

    if (markerMatchAlternative(p.x, p.y))
    {
      alternativeDiagram[p.x][p.y] = voronoiPrune;
    }
    else
    {
      alternativeDiagram[p.x][p.y] = voronoiKeep;
    }
  }

  // //delete worms
  while (!end_cells.empty())
  {
    INTPOINT p = end_cells.front();
    end_cells.pop();

    if (isVoronoiAlternative(p.x, p.y) && getNumVoronoiNeighborsAlternative(p.x, p.y) == 1)
    {
      alternativeDiagram[p.x][p.y] = voronoiPrune;

      for (int dx = -1; dx <= 1; ++dx)
      {
        for (int dy = -1; dy <= 1; ++dy)
        {
          if (!(dx || dy) || (dx && dy))
          {
            continue;
          }
          int nx = p.x + dx;
          int ny = p.y + dy;
          if (nx < 0 || nx >= sizeX || ny < 0 || ny >= sizeY)
          {
            continue;
          }
          if (isVoronoiAlternative(nx, ny))
          {
            if (getNumVoronoiNeighborsAlternative(nx, ny) == 1)
            {
              end_cells.push(INTPOINT(nx, ny));
            }
          }
        }
      }
    }
  }
}

bool DynamicVoronoi::markerMatchAlternative(int x, int y)
{
  // prune if this returns true

  bool f[8];

  int nx, ny;
  int dx, dy;

  int i = 0;
  //  int obstacleCount=0;
  int voroCount = 0;
  for (dy = 1; dy >= -1; dy--)
  {
    ny = y + dy;
    for (dx = -1; dx <= 1; dx++)
    {
      if (dx || dy)
      {
        nx = x + dx;
        int v = alternativeDiagram[nx][ny];
        bool b = (v <= free && v != voronoiPrune);
        //	if (v==occupied) obstacleCount++;
        f[i] = b;
        if (v <= free && !(dx && dy))
          voroCount++;
        i++;
      }
    }
  }

  /*
   * 5 6 7
   * 3   4
   * 0 1 2
   */

  {
    // connected horizontal or vertically to only one cell
    if (voroCount == 1 && (f[1] || f[3] || f[4] || f[6]))
    {
      return false;
    }

    // 4-connected
    if ((!f[0] && f[1] && f[3]) || (!f[2] && f[1] && f[4]) || (!f[5] && f[3] && f[6]) || (!f[7] && f[6] && f[4]))
      return false;

    if ((f[3] && f[4] && !f[1] && !f[6]) || (f[1] && f[6] && !f[3] && !f[4]))
      return false;
  }
  return true;
}

int DynamicVoronoi::getNumVoronoiNeighborsAlternative(int x, int y) const
{
  int count = 0;
  for (int dx = -1; dx <= 1; dx++)
  {
    for (int dy = -1; dy <= 1; dy++)
    {
      if ((dx == 0 && dy == 0) || (dx != 0 && dy != 0))
      {
        continue;
      }

      int nx = x + dx;
      int ny = y + dy;
      if (nx < 0 || nx >= sizeX || ny < 0 || ny >= sizeY)
      {
        continue;
      }
      if (alternativeDiagram[nx][ny] == free || alternativeDiagram[nx][ny] == voronoiKeep)
      {
        count++;
      }
    }
  }
  return count;
}

DynamicVoronoi::markerMatchResult DynamicVoronoi::markerMatch(int x, int y)
{
  // implementation of connectivity patterns
  bool f[8];

  int nx, ny;
  int dx, dy;

  int i = 0;
  int count = 0;
  //  int obstacleCount=0;
  int voroCount = 0;
  int voroCountFour = 0;

  for (dy = 1; dy >= -1; dy--)
  {
    ny = y + dy;
    for (dx = -1; dx <= 1; dx++)
    {
      if (dx || dy)
      {
        nx = x + dx;
        dataCell nc = data[nx][ny];
        int v = nc.voronoi;
        bool b = (v <= free && v != voronoiPrune);
        //	if (v==occupied) obstacleCount++;
        f[i] = b;
        if (b)
        {
          voroCount++;
          if (!(dx && dy))
            voroCountFour++;
        }
        if (b && !(dx && dy))
          count++;
        //	if (v<=free && !(dx && dy)) voroCount++;
        i++;
      }
    }
  }
  if (voroCount < 3 && voroCountFour == 1 && (f[1] || f[3] || f[4] || f[6]))
  {
    //    assert(voroCount<2);
    //    if (voroCount>=2) printf("voro>2 %d %d\n", x, y);
    return keep;
  }

  // 4-connected
  if ((!f[0] && f[1] && f[3]) || (!f[2] && f[1] && f[4]) || (!f[5] && f[3] && f[6]) || (!f[7] && f[6] && f[4]))
    return keep;
  if ((f[3] && f[4] && !f[1] && !f[6]) || (f[1] && f[6] && !f[3] && !f[4]))
    return keep;

  // keep voro cells inside of blocks and retry later
  if (voroCount >= 5 && voroCountFour >= 3 && data[x][y].voronoi != voronoiRetry)
  {
    return retry;
  }

  return pruned;
}
