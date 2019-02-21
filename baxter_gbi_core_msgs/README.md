## baxter_gbi_core_msgs

## Authors

| Name | E-mail |
|------|--------|
| Patrick Roncagliolo | roncapat@gmail.com |

## Package content

This package contains the structure of the message *status.msg*:
```
std_msgs/Header header
string context_type

string m_title
string[] m_options
string[] m_fixed_options
int8 m_selection

string pbr_action
string pbr_msg
```