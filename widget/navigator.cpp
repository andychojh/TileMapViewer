#include <vector>
#include <map>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <cmath>
#include <queue>
#include "WalkNode.h"
#include "Navigator.h"


using namespace std;

// === Utility Functions ===
double Navigator::deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

double Navigator::haversine(double lng1, double lat1, double lng2, double lat2) {
    double dlat = deg2rad(lat2 - lat1);
    double dlng = deg2rad(lng2 - lng1);
    double a = sin(dlat / 2) * sin(dlat / 2) +
        cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
        sin(dlng / 2) * sin(dlng / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return EARTH_RADIUS_M * c;
}

const WalkNode* Navigator::findClosestNode(const vector<WalkNode>& nodes, double lng, double lat)
{

    const WalkNode* closestNode = nullptr;
    double minDistance = std::numeric_limits<double>::max();

    for (const WalkNode& node : nodes) {
        double d = haversine(lng, lat, node.lng, node.lat);
        if (d < minDistance) {
            minDistance = d;
            closestNode = &node;
        }
    }

    return closestNode;
}

const Point* Navigator::findClosestPoint(double lng, double lat)
{

    const Point* closestNode = nullptr;
    double minDistance = std::numeric_limits<double>::max();


    for (Point* pt : points) {  // points: std::vector<Point*>
        double d = haversine(lng, lat, pt->coordinates.first, pt->coordinates.second);
        if (d < minDistance) {
            minDistance = d;
            closestNode = pt;
        }
    }

    /*
        for (auto it = points.begin(); it != points.end(); ++it) {
        double d = haversine(lng, lat, (*it)->coordinates.first, (*it)->coordinates.second);
        if (d < minDistance) {
            minDistance = d;
            closestNode = it;
        }
    }*/
    qDebug() << closestNode->name;
    
    return closestNode;
}


// === Path Computation ===
TurnDirection Navigator::getTurnDirection(const WalkNode& A, const WalkNode& B, const WalkNode& C) {
    double angle= 0.0; 

    if (angle < 30.0) return TurnDirection::STRAIGHT;

    // A에서 B, B에서 C 벡터 계산
    double ab[2] = { B.lng - A.lng, B.lat - A.lat };
    double bc[2] = { C.lng - B.lng, C.lat - B.lat };

    // 두 벡터 사이 각도 계산 - atan2. radian으로 표현.
    angle = atan2(bc[1], bc[0]) - atan2(ab[1], ab[0]);

    // 각도를 -pi ~ pi 범위 조정
    while (angle <= -M_PI) angle += 2 * M_PI;
    while (angle > M_PI) angle -= 2 * M_PI;

    // 360도로 변환
    angle = angle * 180.0 / M_PI;

    // -30도에서 30도 사이는 직진
    if (std::abs(angle) < 30.0) return TurnDirection::STRAIGHT; 
    else if (angle > 0) { // 좌회전
        return TurnDirection::LEFT;
    }
    else { // 우회전
        return TurnDirection::RIGHT;
    }
    
}

// === I/O and Loader ===

vector<Point*> Navigator::load_points(QString& filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qDebug() << "Error: Could not open JSON file: " << file.errorString();
    }

    // JSON 데이터 읽기 
    QByteArray jsonData = file.readAll();
    file.close();

    // JSON 데이터 파싱 
    QJsonParseError jsonError;
    QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData, &jsonError);
    if (jsonError.error != QJsonParseError::NoError) {
        qDebug() << "Error: JSON parse error:" << jsonError.errorString();
    }

    // Json 데이터 접근 & 저장 
    vector<Point*> points;
    if (!jsonDoc.isArray()) {
        qDebug() << "Error: JSON document is not an array";
        return points;
    }

    QJsonArray jsonArray = jsonDoc.array();


    for (const QJsonValue& value : jsonArray) {
        if (!value.isObject()) {
            qDebug() << "Warning: skipping non-object JSON element";
            continue;
        }

        QJsonObject obj = value.toObject();

        Point* pt = nullptr;

        // coordinates (필수)
        if (!obj.contains("coordinates")) {
            qDebug() << "Error: no 'coordinates' field";
            continue;
        }

        QJsonArray arr = obj["coordinates"].toArray();
        if (arr.size() < 2 || !arr[0].isDouble() || !arr[1].isDouble()) {
            qDebug() << "Error: 'coordinates' array invalid";
            continue;
        }

        double lng = arr[0].toDouble();
        double lat = arr[1].toDouble();

        // address와 amenity가 있으면 → Place 생성
        bool isPlace = obj.contains("address") && obj.contains("amenity");

        if (isPlace) {
            Place* place = new Place();
            place->coordinates = { lng, lat };

            if (obj.contains("name")) place->name = obj["name"].toString();
            if (obj.contains("address")) place->address = obj["address"].toString();
            if (obj.contains("amenity")) place->amenity = obj["amenity"].toString();

            pt = place;
        }
        else {
            Point* base = new Point();
            if (obj.contains("name")) base->name = obj["name"].toString();;
            base->coordinates = { lng, lat };
            pt = base;
        }

        points.push_back(pt);
    }

    return points;
}


vector<WalkWay> Navigator::load_walkWay(QString& filePath) {

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qDebug() << "Error: Could not open JSON file:" << file.errorString();
    }

    // JSON 데이터 읽기 
    QByteArray jsonData = file.readAll();
    file.close();

    // JSON 데이터 파싱 
    QJsonParseError jsonError;
    QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData, &jsonError);
    if (jsonError.error != QJsonParseError::NoError) {
        qDebug() << "Error: JSON parse error:" << jsonError.errorString();
    }

    // JSON 데이터 접근 & 저장 
    vector<WalkWay> walkWays;
    if (jsonDoc.isArray()) {
        QJsonArray jsonArray = jsonDoc.array();

        // 배열의 요소에 접근
        for (int i = 0; i < jsonArray.size(); ++i) {
            QJsonValue arrayElement = jsonArray.at(i);
            if (arrayElement.isObject()) {
                QJsonObject elementObject = arrayElement.toObject();
                QJsonValue wayId = elementObject.value("idx");
                QJsonValue wayName_ = elementObject.value("name");
                QString wayName = (!wayName_.isNull() && !wayName_.isUndefined())
                    ? wayName_.toString()
                    : "noname";
                QJsonValue coordinates = elementObject.value("coordinates");
                QJsonValue connections = elementObject.value("connections");

                QJsonArray coordinatesArray = coordinates.toArray();
                vector<pair<double, double>> coordVector;
                coordVector.reserve(coordinatesArray.size());
                for (const QJsonValue& v : coordinatesArray) {
                    coordVector.push_back(pair(v[0].toDouble(), v[1].toDouble()));
                }


                QJsonArray connectionsArray = connections.toArray();
                vector<Connection> conns;
                for (const QJsonValue& con : connectionsArray) {
                    QJsonArray arr = con.toObject().value("linked_indices").toArray();
                    Connection connection;
                    connection.target_wayId = con.toObject().value("way_id").toInt();
                    for (int i = 0; i < 2; i++) connection.linked_indices[i] = arr[i].toInt();
                    conns.push_back(connection);
                }

                // WalkWay 구조체 생성 
                WalkWay walkWay;
                walkWay.name = wayName;
                walkWay.wayId = wayId.toInt();
                walkWay.coordinates = coordVector;
                walkWay.connections = conns;
                walkWays.push_back(walkWay);
            }
        }
        return walkWays;
    }
    else {
        qDebug() << "Error: Root element is not an array.";
    }
}

vector<WalkNode> Navigator::create_nodes(vector<WalkWay>& allWalkWays) {
    vector<WalkNode> walkNodes;

    // 특정 좌표(Key)에 어떤 노드(Value: ID)가 생성되었는지 기록하는 지도입니다.
    // 이걸 사용하면 같은 위치에 노드가 중복으로 생성되는 걸 막을 수 있습니다.
    map<pair<long long, long long>, int> coordinateToNodeIdMap;

    // 지도(map)의 Key로 사용하기 위해, 실수형 좌표를 정수형으로 변환하는 함수입니다.
    // 1e7을 곱해 소수점 7자리까지의 정밀도를 보장합니다.
    auto convertCoordToKey = [](double lng, double lat) {
        return make_pair(static_cast<long long>(lng * 1e7), static_cast<long long>(lat * 1e7));
        };

    // --- 1단계: 모든 도로의 각 지점을 노드로 생성 ---
    // 모든 WalkWay 데이터를 하나씩 순회합니다.
    for (const WalkWay& currentWay : allWalkWays) {
        // 현재 WalkWay를 구성하는 모든 좌표들을 순회합니다.
        for (int i = 0; i < currentWay.coordinates.size(); ++i) {
            const pair<double, double>& currentCoord = currentWay.coordinates[i];
            auto mapKey = convertCoordToKey(currentCoord.first, currentCoord.second);

            // 이 좌표에 해당하는 노드가 이전에 생성된 적이 있는지 확인합니다.
            auto it = coordinateToNodeIdMap.find(mapKey);
            if (it == coordinateToNodeIdMap.end()) {
                // 생성된 적이 없다면, 새로운 노드를 만듭니다.
                WalkNode newNode;
                newNode.id = walkNodes.size(); // 새 ID는 현재까지 생성된 노드의 개수와 같습니다.
                newNode.lng = currentCoord.first;
                newNode.lat = currentCoord.second;
                // 이 노드가 어떤 도로의 몇 번째 지점인지 기록합니다.
                newNode.walkPos.push_back({ currentWay.wayId, i });

                // 새로 생성한 노드를 전체 노드 리스트에 추가합니다.
                walkNodes.push_back(newNode);
                // 지도에 (좌표 -> 새 노드 ID) 정보를 기록하여 다음 중복 확인에 사용합니다.
                coordinateToNodeIdMap[mapKey] = newNode.id;

            }
            else {
                // 이미 존재하는 노드(교차점)라면, 새 노드를 만들지 않습니다.
                int existingNodeId = it->second;
                // 대신, 이 노드가 현재 도로에도 속해있다는 정보만 추가해줍니다.
                walkNodes[existingNodeId].walkPos.push_back({ currentWay.wayId, i });
            }
        }
    }

    // --- 2단계: 같은 도로 내에서 인접한 노드들 연결 ---
    // 모든 WalkWay를 다시 순회하여 노드 간의 이웃 관계를 설정합니다.
    for (const WalkWay& currentWay : allWalkWays) {
        for (int i = 0; i < currentWay.coordinates.size() - 1; ++i) {
            // 현재 지점과 바로 다음 지점의 좌표 키를 가져옵니다.
            auto keyOfCurrentNode = convertCoordToKey(currentWay.coordinates[i].first, currentWay.coordinates[i].second);
            auto keyOfNextNode = convertCoordToKey(currentWay.coordinates[i + 1].first, currentWay.coordinates[i + 1].second);

            // 각 키에 해당하는 노드 ID를 찾습니다.
            int currentNodeId = coordinateToNodeIdMap[keyOfCurrentNode];
            int nextNodeId = coordinateToNodeIdMap[keyOfNextNode];

            // 두 노드를 서로의 'neighbors' 리스트에 추가하여 양방향으로 연결합니다.
            walkNodes[currentNodeId].neighbors.push_back(nextNodeId);
            walkNodes[nextNodeId].neighbors.push_back(currentNodeId);
        }
    }

    // --- 3단계: 서로 다른 도로 사이의 연결점(교차로) 처리 ---
    // `connections` 정보를 바탕으로 도로와 도로를 연결합니다.
    for (const WalkWay& sourceWay : allWalkWays) {
        for (const Connection& connectionInfo : sourceWay.connections) {

            // 연결 대상 도로(targetWay)를 찾습니다.
            const WalkWay* targetWay = nullptr;
            for (const WalkWay& w : allWalkWays) {
                if (w.wayId == connectionInfo.target_wayId) {
                    targetWay = &w;
                    break;
                }
            }

            if (targetWay) {
                // 연결에 참여하는 두 지점의 노드 ID를 각각 찾습니다.
                auto sourceNodeKey = convertCoordToKey(sourceWay.coordinates[connectionInfo.linked_indices[0]].first, sourceWay.coordinates[connectionInfo.linked_indices[0]].second);
                int sourceNodeId = coordinateToNodeIdMap[sourceNodeKey];

                auto targetNodeKey = convertCoordToKey(targetWay->coordinates[connectionInfo.linked_indices[1]].first, targetWay->coordinates[connectionInfo.linked_indices[1]].second);
                int targetNodeId = coordinateToNodeIdMap[targetNodeKey];

                // 중복 연결을 방지하면서, 두 노드를 서로의 이웃으로 추가합니다.
                // sourceNode의 이웃 목록에 targetNode가 없으면 추가합니다.
                if (find(walkNodes[sourceNodeId].neighbors.begin(), walkNodes[sourceNodeId].neighbors.end(), targetNodeId) == walkNodes[sourceNodeId].neighbors.end()) {
                    walkNodes[sourceNodeId].neighbors.push_back(targetNodeId);
                }
                // targetNode의 이웃 목록에 sourceNode가 없으면 추가합니다.
                if (find(walkNodes[targetNodeId].neighbors.begin(), walkNodes[targetNodeId].neighbors.end(), sourceNodeId) == walkNodes[targetNodeId].neighbors.end()) {
                    walkNodes[targetNodeId].neighbors.push_back(sourceNodeId);
                }
            }
        }
    }

    // 완성된 노드 네트워크를 반환합니다.
    return walkNodes;
}

// == 최단경로 알고리즘 == 
vector<int> Navigator::dijkstra(vector<WalkNode>& walkNodes, int start_id, int end_id) {

    for (auto& node : walkNodes) { // Reset fields before each search
        node.dist = numeric_limits<double>::infinity();
        node.prev = -1;
        node.visited = false;
    }

    // Priorirty_queue (가중치, node_id)
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;

    walkNodes[start_id].dist = 0.0;
    pq.push({ 0.0, start_id });

    while (!pq.empty()) {
        pair<double, int> top = pq.top();       // 우선 순위 node explore 
        pq.pop();
        double curr_dist = top.first;
        int curr_id = top.second;
        WalkNode& curr = walkNodes[curr_id];

        if (curr.visited) continue;         // 이미 방문한 node면 다음 우선 순위 노드로 진행 
        curr.visited = true;

        if (curr_id == end_id) break;       // 목적지 node에 도착시 종료 

        //qDebug() << "Explore neighbour of node" << curr_id; 
        for (int nei_id : curr.neighbors) {
            //qDebug() << "Neighbour " << nei_id; 
            // 이웃 순회 
            WalkNode& nei = walkNodes[nei_id];
            double weight = haversine(curr.lng, curr.lat, nei.lng, nei.lat);

            if (nei.dist > curr.dist + weight) {
                // 최단거리 업데이트 
                nei.dist = curr.dist + weight;
                nei.prev = curr_id;
                pq.push({ nei.dist, nei_id });
            }
        }
    }

    // 경로 추적 
    vector<int> path;
    int now = end_id;
    if (walkNodes[now].prev == -1 && now != start_id) {
        qDebug() << "No path found!";
        return path;      // 경로 없음 
    }
    while (now != -1) {
        path.push_back(now);
        now = walkNodes[now].prev;
    }
    reverse(path.begin(), path.end());          // 역추적 한 경로 뒤집기       
    return path;
}

 
// 거쳐간 wayId랑, 이동 거리 구하기 
vector<WayTrack> Navigator::wayTrackerWithDirection(const vector<int>& path, const vector<WalkNode>& walkNodes, const vector<WalkWay>& walkWays) {
    vector<WayTrack> wayTrackLog;

    if (path.size() < 2) return wayTrackLog;

    // 첫 WayTrack 초기화
    WayTrack currentTrack;
    const WalkNode& startNode = walkNodes[path[0]];
    const WalkNode& nextNode = walkNodes[path[1]];

    // 두 노드가 공통으로 속한 wayId 찾기
    int commonWayId = -1;
    for (const auto& pos1 : startNode.walkPos) {
        for (const auto& pos2 : nextNode.walkPos) {
            if (pos1.first == pos2.first) {
                commonWayId = pos1.first;
                break;
            }
        }
        if (commonWayId != -1) break;
    }

    currentTrack.wayId = commonWayId;
    currentTrack.distance = haversine(startNode.lng, startNode.lat, nextNode.lng, nextNode.lat);
    currentTrack.direction = TurnDirection::STRAIGHT; // 시작은 직진
    currentTrack.tpOffset = 0;

    // 경로를 순회하며 WayTrack 생성
    for (size_t i = 1; i < path.size() - 1; ++i) {
        const WalkNode& prev = walkNodes[path[i - 1]];
        const WalkNode& curr = walkNodes[path[i]];
        const WalkNode& next = walkNodes[path[i + 1]];

        TurnDirection turn = getTurnDirection(prev, curr, next);

        // 현재 노드와 다음 노드가 공통으로 속한 wayId 찾기
        int nextWayId = -1;
        for (const auto& pos1 : curr.walkPos) {
            for (const auto& pos2 : next.walkPos) {
                if (pos1.first == pos2.first) {
                    nextWayId = pos1.first;
                    break;
                }
            }
            if (nextWayId != -1) break;
        }

        // 도로가 바뀌거나, 회전이 필요하면 현재 트랙을 저장하고 새 트랙 시작
        if (nextWayId != currentTrack.wayId || turn != TurnDirection::STRAIGHT) {
            wayTrackLog.push_back(currentTrack);

            currentTrack = WayTrack(); // 새 트랙으로 리셋
            currentTrack.wayId = nextWayId;
            currentTrack.direction = turn;
            currentTrack.tpOffset = i;
        }
        currentTrack.distance += haversine(curr.lng, curr.lat, next.lng, next.lat);
    }

    // 마지막 트랙 추가
    wayTrackLog.push_back(currentTrack);

    // 각 WayTrack에 도로 이름 채우기
    for (auto& track : wayTrackLog) {
        for (const auto& way : walkWays) {
            if (track.wayId == way.wayId) {
                track.name = way.name;
                track.hasName = (way.name != "noname");
                break;
            }
        }
    }

    return wayTrackLog;
}


vector<int> Navigator::findPath(double startLng, double startLat, double destLng, double destLat) {
    const WalkNode* startNode = findClosestNode(walkNodes, startLng, startLat);
    const WalkNode* destNode = findClosestNode(walkNodes, destLng, destLat);

    if (!startNode || !destNode) { // 찾을 수 없으면 에러처리.
        qDebug() << "Error: Could not find start or destination node.";
        return vector<int>(); // 빈 벡터
    }

    // 다익스트라 최단 경로 계산
    vector<int> path = dijkstra(walkNodes, startNode->id, destNode->id);

    return path;
}

void Navigator::run() {
    QString filePath = "Seocho_map_cooked.json";
    QString pointPath = "points.json";
    points = load_points(pointPath);
    walkWays = load_walkWay(filePath);
    walkNodes = create_nodes(walkWays);
}
