export const importSnippetsObj = {
  _cv2: [
    {
      type: "method",
      label: "imread(image_path, cv2.IMREAD_COLOR)",
      code: "imread(image_path, cv2.IMREAD_COLOR)",
      descriptions: "This method is used to read an image from its path.",
    },
    {
      type: "method",
      label: "imread(image_path, cv2.IMREAD_GRAYSCALE)",
      code: "imread(image_path, cv2.IMREAD_GRAYSCALE)",
      descriptions: "This method is used to read an image from its path.",
    },
    {
      type: "method",
      label: "imread(image_path, cv2.IMREAD_UNCHANGED)",
      code: "imread(image_path, cv2.IMREAD_UNCHANGED)",
      descriptions: "This method is used to read an image from its path.",
    },
    {
      type: "method",
      label: "imshow(window_name, image)",
      code: "imshow(window_name, image)",
      descriptions: "It is used to show the image in the window.",
    },
    {
      type: "method",
      label: "imwrite(filename, image)",
      code: "imwrite(filename, image)",
      descriptions:
        "This method is used to write or save an image using OpenCV.",
    },
    {
      type: "method",
      label:
        "line(image, start_coordinates, end_coordinates, color_in_bgr, line_thickness)",
      code: "line(image, start_coordinates, end_coordinates, color_in_bgr, line_thickness)",
      descriptions:
        "By using this function we can create a line in the image from start coordinates to end coordinates with a certain thickness which can be mentioned specifically as well.",
    },
    {
      type: "method",
      label:
        "rectangle(image,top_left_vertex_coordinates, lower_right_vertex_coordinates, color_in_bgr, thickness)",
      code: "rectangle(image,top_left_vertex_coordinates, lower_right_vertex_coordinates, color_in_bgr, thickness)",
      descriptions:
        "This function is used to create a box with a certain thickness and color which can be specified as well.",
    },
    {
      type: "method",
      label: "circle(image, center_coordinates, radius, color, thickness)",
      code: "circle(image, center_coordinates, radius, color, thickness)",
      descriptions:
        "It is used to draw a circle whose centre and radius length is given with a certain thickness and the colour of the strokes of the circle.",
    },
    {
      type: "method",
      label: "polylines(image, [pts], isClosed, color, thickness)",
      code: "polylines(image, [pts], isClosed, color, thickness)",
      descriptions:
        "It is used to draw a polygon on any image whose vertex coordinates are provided.",
    },
    {
      type: "method",
      label:
        "putText(image, ‘TextContent’, (‘text_starting_point_coordinates’)",
      code: "putText(image, ‘TextContent’, (‘text_starting_point_coordinates’)",
      descriptions: "",
    },
    {
      type: "method",
      label: "add(image1, image2)",
      code: "add(image1, image2)",
      descriptions: "This function is used to add two images.",
    },
    {
      type: "method",
      label: "subtract(image1, image2)",
      code: "subtract(image1, image2)",
      descriptions: "This function is used to subtract two images.",
    },
    {
      type: "method",
      label: "addWeighted(image1, weight1, image2, weight2, gammaValue)",
      code: "addWeighted(image1, weight1, image2, weight2, gammaValue)",
      descriptions:
        "This is also known as Alpha Blending. This is nothing but a weighted blending process of two images.",
    },
    {
      type: "method",
      label: "bitwise_and(image1, image2, destination, mask)",
      code: "bitwise_and(image1, image2, destination, mask)",
      descriptions:
        "This performs bitwise and logical operations between two images.",
    },
    {
      type: "method",
      label: "bitwise_or(image1, image2, destination, mask)",
      code: "bitwise_or(image1, image2, destination, mask)",
      descriptions:
        "This performs bitwise or logical operations between two images.",
    },
    {
      type: "method",
      label: "bitwise_not(image, destination, mask)",
      code: "bitwise_not(image, destination, mask)",
      descriptions:
        "This performs bitwise not logical operations between an image and a mask.",
    },
    {
      type: "method",
      label: "bitwise_xor(image1, image2, destination, mask)",
      code: "bitwise_xor(image1, image2, destination, mask)",
      descriptions:
        "This performs bitwise xor logical operations between two images.",
    },
    {
      type: "method",
      label: "inRange(raw_image, lower, upper)",
      code: "inRange(raw_image, lower, upper)",
      descriptions: "",
    },
    {
      type: "method",
      label: "cvtColor(image, cv2.COLOR_BGR2GRAY)",
      code: "cvtColor(image, cv2.COLOR_BGR2GRAY)",
      descriptions: "",
    },
    {
      type: "method",
      label: "cvtColor(img, cv2.COLOR_BGR2HSV)",
      code: "cvtColor(img, cv2.COLOR_BGR2HSV)",
      descriptions: "",
    },
    {
      type: "method",
      label: "cvtColor(img, cv2.COLOR_BGR2LAB)",
      code: "cvtColor(img, cv2.COLOR_BGR2LAB)",
      descriptions: "",
    },
  ],
};
