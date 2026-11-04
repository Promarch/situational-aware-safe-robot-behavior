#include "nanoleaf.h"

#include <arpa/inet.h>
#include <curl/curl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <stdexcept>

namespace {
size_t appendBody(char* data, size_t size, size_t n, void* out) {
  static_cast<std::string*>(out)->append(data, size * n);
  return size * n;
}

void put16(std::vector<uint8_t>& f, uint16_t v) {
  f.push_back(static_cast<uint8_t>(v >> 8));
  f.push_back(static_cast<uint8_t>(v & 0xFF));
}
}  // namespace

Nanoleaf::Nanoleaf(std::string ip, std::string token, std::vector<uint16_t> panels,
                   uint16_t stream_port)
    : ip_(std::move(ip)), token_(std::move(token)), panels_(std::move(panels)) {
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) throw std::runtime_error("nanoleaf: cannot create a UDP socket");
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(stream_port);
  if (::inet_pton(AF_INET, ip_.c_str(), &addr.sin_addr) != 1 ||
      ::connect(fd_, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
    ::close(fd_);
    throw std::runtime_error("nanoleaf: bad address " + ip_);
  }
}

Nanoleaf::~Nanoleaf() {
  if (fd_ >= 0) ::close(fd_);
}

bool Nanoleaf::activateStreaming(std::string* error, long timeout_s) const {
  const std::string url = "http://" + ip_ + ":16021/api/v1/" + token_ + "/effects";
  const std::string body =
      R"({"write":{"command":"display","animType":"extControl","extControlVersion":"v2"}})";

  CURL* curl = curl_easy_init();
  if (!curl) {
    if (error) *error = "curl_easy_init failed";
    return false;
  }
  std::string response;
  curl_slist* headers = curl_slist_append(nullptr, "Content-Type: application/json");
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout_s);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);  // no SIGALRM for the timeout (threads)
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, appendBody);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
  const CURLcode rc = curl_easy_perform(curl);
  long status = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (rc != CURLE_OK) {
    if (error) *error = std::string("no answer from ") + ip_ + ": " + curl_easy_strerror(rc);
    return false;
  }
  if (status < 200 || status >= 300) {
    if (error) {
      *error = "HTTP " + std::to_string(status) +
               (status == 401 ? " (token not accepted)" : "") +
               (response.empty() ? "" : ": " + response);
    }
    return false;
  }
  return true;
}

void Nanoleaf::setAll(Rgb colour, uint16_t transition_100ms) const {
  std::vector<uint8_t> f;
  f.reserve(2 + 8 * panels_.size());
  put16(f, static_cast<uint16_t>(panels_.size()));
  for (uint16_t id : panels_) {
    put16(f, id);
    f.insert(f.end(), {colour.r, colour.g, colour.b, 0});  // W = 0
    put16(f, transition_100ms);
  }
  send(f);
}

void Nanoleaf::setPanels(const std::vector<std::pair<uint16_t, Rgb>>& colours,
                         uint16_t transition_100ms) const {
  std::vector<uint8_t> f;
  f.reserve(2 + 8 * colours.size());
  put16(f, static_cast<uint16_t>(colours.size()));
  for (const auto& [id, c] : colours) {
    put16(f, id);
    f.insert(f.end(), {c.r, c.g, c.b, 0});
    put16(f, transition_100ms);
  }
  send(f);
}

void Nanoleaf::send(const std::vector<uint8_t>& frame) const {
  // Errors (no route, ECONNREFUSED after an ICMP unreachable, full buffer) are ignored.
  (void)::send(fd_, frame.data(), frame.size(), MSG_DONTWAIT);
}
