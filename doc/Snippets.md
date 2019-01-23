# Snippets

```cpp
  // Advertise the Model message on '~/model/info'
  this->data->modelPub = this->node->Advertise<msgs::Model>("~/model/info");

  // Capture the original dimensions of the models
  std::regex e("(wavebox)(.*)");
  for (auto const model : this->world->Models())
  {
    std::string name = model->GetName();
    if (std::regex_match(name,e))
    {
    }
  }
```