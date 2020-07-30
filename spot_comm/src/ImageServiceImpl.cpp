#include <spot_comm/ImageServiceImpl.h>


  Status ListImageSources(ServerContext* context, const ListImageSourcesRequest* request,
                  ListImageSourcesResponse* response) {
    //create response to list all the possible image sources
    ImageSource* imgsrc = response->add_image_sources();
    imgsrc->set_name("imageName");
    imgsrc->set_rows(480);
    imgsrc->set_cols(640);
    imgsrc->set_depth_scale(1000);
    imgsrc->set_image_type(ImageSource::IMAGE_TYPE_VISUAL);
    
    return Status::OK;
  }

  Status GetImage(ServerContext* context, const GetImageRequest* request,
                  GetImageResponse* response) {

      
      ImageResponse* img = response->add_image_responses();
      ImageCapture* shot;
      ImageSource* imgsrc;
      img->set_allocated_shot(shot);
      img->set_allocated_source(imgsrc);
      img->set_status(ImageResponse::STATUS_OK);
      return Status::OK;
      
  }
